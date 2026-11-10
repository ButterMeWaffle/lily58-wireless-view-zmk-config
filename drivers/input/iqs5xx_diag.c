/*
 * Temporary bring-up diagnostics for the IQS5xx bus. See iqs5xx_diag.h.
 *
 * Part 1 releases the TWIM peripheral and checks the SDA/SCL pins as plain GPIO: idle levels with
 * different pulls (does the adapter have its own pull-ups?), shorts between the two lines, and how
 * fast each line rises (the capacitance it sees).
 * Part 2 times I2C transfers: a quick failure is a NACK, a ~500 ms failure means the transfer never
 * completed (stuck or odd bus).
 */

#define DT_DRV_COMPAT azoteq_iqs5xx

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_twim.h>

#include "iqs5xx_diag.h"

LOG_MODULE_DECLARE(iqs5xx, CONFIG_INPUT_LOG_LEVEL);

/* Must match &i2c0_default in config/lily58_right.overlay. */
#define DIAG_SDA NRF_GPIO_PIN_MAP(0, 17)
#define DIAG_SCL NRF_GPIO_PIN_MAP(0, 20)
#define DIAG_TWIM ((NRF_TWIM_Type *)DT_REG_ADDR(DT_INST_BUS(0)))

#define RISE_SAMPLES 32
#define RISE_LIMIT_US 50
#define PROBE_TRIES 8
#define SLOW_MS 100
#define SWEEP_FIRST 0x08
#define SWEEP_LAST 0x77
#define MAX_ODD 8

struct probe_stat {
    int first_err;
    uint16_t acks;
    uint32_t min_us;
    uint32_t max_us;
    uint32_t sum_us;
};

static struct {
    bool done;

    /* [0] = SDA, [1] = SCL */
    uint8_t lvl_pullup[2];
    uint8_t lvl_pulldown[2];
    uint8_t lvl_float[2];
    uint8_t other_when_driven_low[2];
    uint32_t rise_min_ns[2];
    uint32_t rise_max_ns[2];
    uint32_t rise_sum_ns[2];
    uint8_t rise_timeouts[2];
    uint8_t after[2];

    uint16_t sweep_ack;
    uint16_t sweep_fast;
    uint16_t sweep_slow;
    uint32_t sweep_fast_sum_us;
    uint32_t sweep_slow_sum_ms;
    uint8_t odd[MAX_ODD];
    uint8_t odd_count;
    bool odd_are_slow;

    struct probe_stat zero_write;
    struct probe_stat write1;
    struct probe_stat read1;
    struct probe_stat write_read;
} res;

static const uint32_t pins[2] = {DIAG_SDA, DIAG_SCL};

static uint32_t cyc_to_ns(uint32_t cycles) { return (uint32_t)k_cyc_to_ns_floor64(cycles); }

static uint32_t cyc_to_us(uint32_t cycles) { return (uint32_t)k_cyc_to_us_floor64(cycles); }

static uint8_t read_pulled(uint32_t pin, nrf_gpio_pin_pull_t pull) {
    nrf_gpio_cfg_input(pin, pull);
    k_busy_wait(300);
    return nrf_gpio_pin_read(pin) ? 1 : 0;
}

static void gpio_tests(void) {
    uint32_t saved[2];

    for (int i = 0; i < 2; i++) {
        saved[i] = NRF_P0->PIN_CNF[((pins[i]) & 0x1F)];
    }

    /* Let go of the pins: they are plain GPIO while the TWIM is disabled. */
    nrf_twim_disable(DIAG_TWIM);

    for (int i = 0; i < 2; i++) {
        res.lvl_pullup[i] = read_pulled(pins[i], NRF_GPIO_PIN_PULLUP);
        res.lvl_pulldown[i] = read_pulled(pins[i], NRF_GPIO_PIN_PULLDOWN);
        res.lvl_float[i] = read_pulled(pins[i], NRF_GPIO_PIN_NOPULL);
    }

    /* Drive one line low, see whether the other follows it (short between SDA and SCL). */
    for (int i = 0; i < 2; i++) {
        int other = 1 - i;

        nrf_gpio_cfg_input(pins[other], NRF_GPIO_PIN_PULLUP);
        nrf_gpio_cfg_output(pins[i]);
        nrf_gpio_pin_clear(pins[i]);
        k_busy_wait(300);
        res.other_when_driven_low[i] = nrf_gpio_pin_read(pins[other]) ? 1 : 0;
        nrf_gpio_cfg_input(pins[i], NRF_GPIO_PIN_PULLUP);
    }

    /* Rise time from low to high through the internal pull-up. */
    for (int i = 0; i < 2; i++) {
        uint32_t pin = pins[i];
        uint32_t reg = ((pin) & 0x1F);

        nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_PULLUP);
        uint32_t cnf_in = NRF_P0->PIN_CNF[reg];

        nrf_gpio_cfg_output(pin);
        uint32_t cnf_out = NRF_P0->PIN_CNF[reg];

        res.rise_min_ns[i] = UINT32_MAX;
        for (int s = 0; s < RISE_SAMPLES; s++) {
            uint32_t t, limit = k_us_to_cyc_ceil32(RISE_LIMIT_US);
            unsigned int key;

            NRF_P0->PIN_CNF[reg] = cnf_out;
            nrf_gpio_pin_clear(pin);
            k_busy_wait(100);

            key = irq_lock();
            uint32_t t0 = k_cycle_get_32();
            NRF_P0->PIN_CNF[reg] = cnf_in;
            while (!nrf_gpio_pin_read(pin) && (k_cycle_get_32() - t0) < limit) {
            }
            t = k_cycle_get_32() - t0;
            irq_unlock(key);

            if (t >= limit) {
                res.rise_timeouts[i]++;
            }
            uint32_t ns = cyc_to_ns(t);
            res.rise_min_ns[i] = MIN(res.rise_min_ns[i], ns);
            res.rise_max_ns[i] = MAX(res.rise_max_ns[i], ns);
            res.rise_sum_ns[i] += ns;
        }
    }

    /* Hand the pins back to the TWIM exactly as they were. */
    for (int i = 0; i < 2; i++) {
        NRF_P0->PIN_CNF[((pins[i]) & 0x1F)] = saved[i];
    }
    nrf_twim_enable(DIAG_TWIM);
}

static void probe_stat_add(struct probe_stat *s, int ret, uint32_t us, bool first) {
    if (first) {
        s->first_err = ret;
        s->min_us = UINT32_MAX;
    }
    if (ret == 0) {
        s->acks++;
    }
    s->min_us = MIN(s->min_us, us);
    s->max_us = MAX(s->max_us, us);
    s->sum_us += us;
}

static void i2c_tests(const struct iqs5xx_config *config) {
    const struct device *bus = config->i2c.bus;
    uint8_t cls[SWEEP_LAST + 1] = {};
    uint8_t buf[2] = {};
    uint8_t reg[2] = {0x00, 0x00};
    uint16_t slow_n = 0;

    for (uint8_t addr = SWEEP_FIRST; addr <= SWEEP_LAST; addr++) {
        uint32_t t0 = k_cycle_get_32();
        int ret = i2c_read(bus, buf, 1, addr);
        uint32_t us = cyc_to_us(k_cycle_get_32() - t0);

        if (ret == 0) {
            res.sweep_ack++;
        }
        if (us >= SLOW_MS * 1000) {
            res.sweep_slow++;
            res.sweep_slow_sum_ms += us / 1000;
            cls[addr] = 2;
            slow_n++;
        } else {
            res.sweep_fast++;
            res.sweep_fast_sum_us += us;
            cls[addr] = 1;
        }
    }

    /* Addresses that behave differently from the majority. */
    res.odd_are_slow = res.sweep_slow < res.sweep_fast;
    for (uint8_t addr = SWEEP_FIRST; addr <= SWEEP_LAST && res.odd_count < MAX_ODD; addr++) {
        if ((res.odd_are_slow && cls[addr] == 2) || (!res.odd_are_slow && cls[addr] == 1)) {
            res.odd[res.odd_count++] = addr;
        }
    }

    for (int n = 0; n < PROBE_TRIES; n++) {
        struct probe_stat *stats[4] = {&res.zero_write, &res.write1, &res.read1, &res.write_read};

        for (int k = 0; k < 4; k++) {
            uint32_t t0 = k_cycle_get_32();
            int ret;

            switch (k) {
            case 0:
                ret = i2c_write(bus, NULL, 0, config->i2c.addr);
                break;
            case 1:
                ret = i2c_write(bus, reg, 1, config->i2c.addr);
                break;
            case 2:
                ret = i2c_read(bus, buf, 1, config->i2c.addr);
                break;
            default:
                ret = i2c_write_read(bus, config->i2c.addr, reg, 2, buf, 2);
                break;
            }
            probe_stat_add(stats[k], ret, cyc_to_us(k_cycle_get_32() - t0), n == 0);
            k_msleep(2);
        }
    }

    for (int i = 0; i < 2; i++) {
        res.after[i] = nrf_gpio_pin_read(pins[i]) ? 1 : 0;
    }
}

void iqs5xx_diag_run(const struct iqs5xx_config *config) {
    LOG_INF("DIAG: starting GPIO and I2C tests");
    gpio_tests();
    i2c_tests(config);
    res.done = true;
}

static void report_probe(const char *name, const struct probe_stat *s) {
    LOG_INF("DIAG i2c 0x74 %s: first_err=%d acks=%u/%d time_us min/avg/max=%u/%u/%u", name,
            s->first_err, s->acks, PROBE_TRIES, s->min_us, s->sum_us / PROBE_TRIES, s->max_us);
}

void iqs5xx_diag_report(void) {
    if (!res.done) {
        return;
    }

    LOG_INF("DIAG gpio idle level (1=high): pullup SDA=%u SCL=%u | pulldown SDA=%u SCL=%u | "
            "no pull SDA=%u SCL=%u",
            res.lvl_pullup[0], res.lvl_pullup[1], res.lvl_pulldown[0], res.lvl_pulldown[1],
            res.lvl_float[0], res.lvl_float[1]);
    LOG_INF("DIAG gpio short test: SDA driven low -> SCL reads %u, SCL driven low -> SDA reads %u "
            "(both should be 1)",
            res.other_when_driven_low[0], res.other_when_driven_low[1]);
    LOG_INF("DIAG gpio rise time ns (min/avg/max, %d samples, %u timeouts): SDA %u/%u/%u | "
            "SCL %u/%u/%u",
            RISE_SAMPLES, res.rise_timeouts[0] + res.rise_timeouts[1], res.rise_min_ns[0],
            res.rise_sum_ns[0] / RISE_SAMPLES, res.rise_max_ns[0], res.rise_min_ns[1],
            res.rise_sum_ns[1] / RISE_SAMPLES, res.rise_max_ns[1]);
    LOG_INF("DIAG i2c sweep 0x%02x-0x%02x 1-byte read: ack=%u fast(<%dms)=%u slow=%u "
            "fast_avg=%uus slow_avg=%ums",
            SWEEP_FIRST, SWEEP_LAST, res.sweep_ack, SLOW_MS, res.sweep_fast, res.sweep_slow,
            res.sweep_fast ? res.sweep_fast_sum_us / res.sweep_fast : 0,
            res.sweep_slow ? res.sweep_slow_sum_ms / res.sweep_slow : 0);
    for (int i = 0; i < res.odd_count; i++) {
        LOG_INF("DIAG i2c sweep minority (%s) address 0x%02x", res.odd_are_slow ? "slow" : "fast",
                res.odd[i]);
    }
    report_probe("zero-length write", &res.zero_write);
    report_probe("1-byte write", &res.write1);
    report_probe("1-byte read", &res.read1);
    report_probe("write 2 + read 2", &res.write_read);
    LOG_INF("DIAG gpio after tests: SDA=%u SCL=%u (1=high)", res.after[0], res.after[1]);
}

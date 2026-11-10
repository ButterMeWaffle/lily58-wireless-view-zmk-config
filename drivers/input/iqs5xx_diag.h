/*
 * Temporary bring-up diagnostics for the IQS5xx bus. Delete together with iqs5xx_diag.c once the
 * touchpad works.
 */

#pragma once

#include "iqs5xx.h"

/* Test the SDA/SCL pins as plain GPIO, then time I2C transfers. Blocks for up to ~1 minute. */
void iqs5xx_diag_run(const struct iqs5xx_config *config);

/* Log the stored results (cheap, call on every setup retry so a late USB capture still sees them). */
void iqs5xx_diag_report(void);

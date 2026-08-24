/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4_touch.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/gt9xx.h>

#include "espressif/esp_i2c.h"

#ifdef CONFIG_INPUT_GT9XX

/* ESP32-P4 Function EV Board: GT911 at I2C0 address 0x5d,
 * SCL GPIO8, SDA GPIO7.  The official Espressif BSP defines both the touch
 * interrupt and reset pins as not connected.  GPIO23 is the LCD backlight,
 * not a touch interrupt.  Touch data is therefore read by polling.
 */

#define TP_I2C_PORT 0
#define TP_I2C_ADDR 0x5d
static int tp_irq_attach(const struct gt9xx_board_s *state, xcpt_t isr,
                         FAR void *arg)
{
  return OK;
}

static void tp_irq_enable(const struct gt9xx_board_s *state, bool enable)
{
}

static int tp_set_power(const struct gt9xx_board_s *state, bool on)
{
  /* The display subboard powers the touch controller permanently. */

  return OK;
}

static const struct gt9xx_board_s g_gt9xx_board =
{
  .irq_attach = tp_irq_attach,
  .irq_enable = tp_irq_enable,
  .set_power  = tp_set_power,
};

int board_touch_initialize(void)
{
  FAR struct i2c_master_s *i2c;

  i2c = esp_i2cbus_initialize(TP_I2C_PORT);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "Touch: failed to initialize I2C%d\n", TP_I2C_PORT);
      return -ENODEV;
    }

  return gt9xx_register("/dev/input0", i2c, TP_I2C_ADDR, &g_gt9xx_board);
}

#endif /* CONFIG_INPUT_GT9XX */

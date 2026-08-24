/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_usbserial.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <nuttx/debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#  include <nuttx/fs/ioctl.h>
#endif

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/serial/serial.h>
#include <arch/irq.h>

#include "riscv_internal.h"

#include "esp_config.h"
#include "esp_irq.h"

#include "esp_private/periph_ctrl.h"
#include "hal/uart_hal.h"
#include "hal/usb_serial_jtag_ll.h"

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

#if !SOC_RCC_IS_INDEPENDENT
#define USJ_RCC_ATOMIC() PERIPH_RCC_ATOMIC()
#else
#define USJ_RCC_ATOMIC()
#endif

/* The hardware buffer has a fixed size of 64 bytes */

#define ESP_USBCDC_BUFFERSIZE 64

/* A connected full-speed USB host emits one SOF packet every millisecond.
 * Allow a generous scheduling margin before declaring the host absent.
 */

#define ESP_USBSERIAL_SOF_GRACE_TICKS (MSEC2TICK(20) + 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_priv_s
{
  const uint8_t  source;        /* Source ID */
  const uint8_t  irq;           /* IRQ number assigned to the source */
  int            cpuint;        /* CPU interrupt assigned */
  clock_t        last_sof;      /* Last observed USB start-of-frame */
  bool           connected;     /* Host is actively producing SOFs */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp_interrupt(int irq, void *context, void *arg);
static bool esp_connected(struct esp_priv_s *priv);

extern uart_dev_t g_uart_usbserial;

/* Serial driver methods */

static int  esp_setup(struct uart_dev_s *dev);
static void esp_shutdown(struct uart_dev_s *dev);
static int  esp_attach(struct uart_dev_s *dev);
static void esp_detach(struct uart_dev_s *dev);
static void esp_txint(struct uart_dev_s *dev, bool enable);
static void esp_rxint(struct uart_dev_s *dev, bool enable);
static bool esp_rxavailable(struct uart_dev_s *dev);
static bool esp_txready(struct uart_dev_s *dev);
static void esp_send(struct uart_dev_s *dev, int ch);
static int  esp_receive(struct uart_dev_s *dev, unsigned int *status);
static int  esp_ioctl(struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_rxbuffer[ESP_USBCDC_BUFFERSIZE];
static char g_txbuffer[ESP_USBCDC_BUFFERSIZE];

static struct esp_priv_s g_usbserial_priv =
{
  .source = ETS_USB_SERIAL_JTAG_INTR_SOURCE,
  .irq    = ESP_SOURCE2IRQ(ETS_USB_SERIAL_JTAG_INTR_SOURCE),
  .cpuint = -ENOMEM,
  .last_sof = 0,
  .connected = false,
};

static struct uart_ops_s g_uart_ops =
{
  .setup       = esp_setup,
  .shutdown    = esp_shutdown,
  .attach      = esp_attach,
  .detach      = esp_detach,
  .txint       = esp_txint,
  .rxint       = esp_rxint,
  .rxavailable = esp_rxavailable,
  .txready     = esp_txready,
  .txempty     = NULL,
  .send        = esp_send,
  .receive     = esp_receive,
  .ioctl       = esp_ioctl,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

uart_dev_t g_uart_usbserial =
{
#ifdef CONFIG_ESPRESSIF_USBSERIAL_CONSOLE
  .isconsole = true,
#else
  .isconsole = false,
#endif
#ifdef CONFIG_SERIAL_REMOVABLE
  .disconnected = true,
#endif
  .recv =
    {
      .size = ESP_USBCDC_BUFFERSIZE,
      .buffer = g_rxbuffer,
    },
  .xmit =
    {
      .size = ESP_USBCDC_BUFFERSIZE,
      .buffer = g_txbuffer,
    },
  .ops = &g_uart_ops,
  .priv = &g_usbserial_priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_connected
 *
 * Description:
 *   Track host presence using USB SOF packets, following Espressif's
 *   connection monitor.  FIFO writability alone is insufficient: after a
 *   packet is committed with no host, the endpoint remains permanently
 *   non-writable and a blocking console writer can stop the system.
 ****************************************************************************/

static bool esp_connected(struct esp_priv_s *priv)
{
  irqstate_t flags;
  clock_t now;
  bool connected;

  now = clock_systime_ticks();
  flags = enter_critical_section();

  if ((usb_serial_jtag_ll_get_intraw_mask() &
       USB_SERIAL_JTAG_INTR_SOF) != 0)
    {
      usb_serial_jtag_ll_clr_intsts_mask(USB_SERIAL_JTAG_INTR_SOF);
      priv->last_sof = now;
      priv->connected = true;
    }
  else if (priv->connected &&
           (clock_t)(now - priv->last_sof) >
           ESP_USBSERIAL_SOF_GRACE_TICKS)
    {
      priv->connected = false;
    }

  connected = priv->connected;

#ifdef CONFIG_SERIAL_REMOVABLE
  g_uart_usbserial.disconnected = !connected;
#endif

  /* With no host, shut down both console data interrupt sources.  Keep only
   * SOF armed so the first frame from a newly attached host can wake the
   * driver and restore the console automatically.
   */

  if (priv->cpuint >= 0)
    {
      if (connected)
        {
          usb_serial_jtag_ll_disable_intr_mask(USB_SERIAL_JTAG_INTR_SOF);
          usb_serial_jtag_ll_ena_intr_mask(
            USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT);
        }
      else
        {
          usb_serial_jtag_ll_disable_intr_mask(
            USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY |
            USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT);
          usb_serial_jtag_ll_ena_intr_mask(USB_SERIAL_JTAG_INTR_SOF);
        }
    }

  leave_critical_section(flags);
  return connected;
}

/****************************************************************************
 * Name: esp_interrupt
 *
 * Description:
 *   This is the common UART interrupt handler. It will be invoked when an
 *   interrupt is received on the 'irq'. It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers. The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int esp_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct esp_priv_s *priv = dev->priv;
  uint32_t int_status = usb_serial_jtag_ll_get_intsts_mask();

  /* A disconnected driver enables only SOF.  The first frame proves that a
   * host is present; switch back to the normal console interrupt sources.
   */

  if ((int_status & USB_SERIAL_JTAG_INTR_SOF) != 0)
    {
      usb_serial_jtag_ll_clr_intsts_mask(USB_SERIAL_JTAG_INTR_SOF);
      priv->last_sof = clock_systime_ticks();
      priv->connected = true;
#ifdef CONFIG_SERIAL_REMOVABLE
      dev->disconnected = false;
#endif
      usb_serial_jtag_ll_disable_intr_mask(USB_SERIAL_JTAG_INTR_SOF);
      usb_serial_jtag_ll_ena_intr_mask(
        USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT);
    }

  /* Send buffer has room and can accept new data. */

  if ((int_status & USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY) != 0)
    {
      usb_serial_jtag_ll_clr_intsts_mask(
        USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY);
      uart_xmitchars(dev);
    }

  /* Data from the host are available to read. */

  if ((int_status & USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT) != 0)
    {
      usb_serial_jtag_ll_clr_intsts_mask(
        USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT);
      uart_recvchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: esp_setup
 *
 * Description:
 *   This method is called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int esp_setup(struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: esp_shutdown
 *
 * Description:
 *   This method is called when the serial port is closed.
 *
 ****************************************************************************/

static void esp_shutdown(struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: esp_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void esp_txint(struct uart_dev_s *dev, bool enable)
{
  struct esp_priv_s *priv = dev->priv;

  if (enable)
    {
      if (esp_connected(priv))
        {
          usb_serial_jtag_ll_txfifo_flush();
          usb_serial_jtag_ll_ena_intr_mask(
            USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY);
        }
      else
        {
          /* A console write must never wait for a USB host.  Make txready()
           * report ready and synchronously consume the software queue;
           * esp_send() discards its bytes while disconnected.
           */

          usb_serial_jtag_ll_disable_intr_mask(
            USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY);
          uart_xmitchars(dev);
        }
    }
  else
    {
      usb_serial_jtag_ll_disable_intr_mask(
        USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY);

      if (esp_connected(priv))
        {
          usb_serial_jtag_ll_txfifo_flush();
        }
    }
}

/****************************************************************************
 * Name: esp_rxint
 *
 * Description:
 *   Call to enable or disable RXRDY interrupts
 *
 ****************************************************************************/

static void esp_rxint(struct uart_dev_s *dev, bool enable)
{
  struct esp_priv_s *priv = dev->priv;

  if (enable)
    {
      if (esp_connected(priv))
        {
          usb_serial_jtag_ll_ena_intr_mask(
            USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT);
        }
      else
        {
          usb_serial_jtag_ll_disable_intr_mask(
            USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT);
          usb_serial_jtag_ll_ena_intr_mask(USB_SERIAL_JTAG_INTR_SOF);
        }
    }
  else
    {
      usb_serial_jtag_ll_disable_intr_mask(
        USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT);
    }
}

/****************************************************************************
 * Name: esp_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode. This method
 *   is called when the serial port is opened. Normally, this is just after
 *   the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling). The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int esp_attach(struct uart_dev_s *dev)
{
  struct esp_priv_s *priv = dev->priv;

  DEBUGASSERT(priv->cpuint == -ENOMEM);

  USJ_RCC_ATOMIC()
    {
      usb_serial_jtag_ll_enable_bus_clock(true);
    }

  usb_serial_jtag_ll_phy_set_defaults();

  /* Try to attach the IRQ to a CPU int */

  priv->cpuint = esp_setup_irq(priv->source,
                               ESP_IRQ_PRIORITY_DEFAULT,
                               ESP_IRQ_TRIGGER_LEVEL,
                               esp_interrupt,
                               dev);
  if (priv->cpuint < 0)
    {
      return priv->cpuint;
    }

  /* Attach and enable the IRQ */

  if (priv->cpuint >= 0)
    {
      up_enable_irq(priv->irq);

      if (esp_connected(priv))
        {
          usb_serial_jtag_ll_ena_intr_mask(
            USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT);
        }
      else
        {
          usb_serial_jtag_ll_ena_intr_mask(USB_SERIAL_JTAG_INTR_SOF);
        }
    }
  else
    {
      up_disable_irq(priv->irq);
    }

  return OK;
}

/****************************************************************************
 * Name: esp_detach
 *
 * Description:
 *   Detach UART interrupts. This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void esp_detach(struct uart_dev_s *dev)
{
  struct esp_priv_s *priv = dev->priv;

  DEBUGASSERT(priv->cpuint != -ENOMEM);

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
  esp_teardown_irq(priv->source, priv->cpuint);

  priv->cpuint = -ENOMEM;
}

/****************************************************************************
 * Name: esp_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

static bool esp_rxavailable(struct uart_dev_s *dev)
{
  return (bool)usb_serial_jtag_ll_rxfifo_data_available();
}

/****************************************************************************
 * Name: esp_txready
 *
 * Description:
 *   Return true if the transmit holding register is empty (TXRDY)
 *
 ****************************************************************************/

static bool esp_txready(struct uart_dev_s *dev)
{
  struct esp_priv_s *priv = dev->priv;

  if (!esp_connected(priv))
    {
      return true;
    }

  return (bool)usb_serial_jtag_ll_txfifo_writable();
}

/****************************************************************************
 * Name: esp_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void esp_send(struct uart_dev_s *dev, int ch)
{
  struct esp_priv_s *priv = dev->priv;

  if (!esp_connected(priv))
    {
      return;
    }

  /* Write the character to the buffer. */

  uint8_t buf[1] = {
    (uint8_t)ch
  };

  usb_serial_jtag_ll_write_txfifo(buf, sizeof(buf));

  /* Flush the character out. */

  usb_serial_jtag_ll_txfifo_flush();
}

/****************************************************************************
 * Name: esp32_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one character.
 *
 ****************************************************************************/

static int esp_receive(struct uart_dev_s *dev, unsigned int *status)
{
  uint8_t buf[1] = {
    0
  };

  *status = 0;
  usb_serial_jtag_ll_read_rxfifo(buf, sizeof(buf));

  return (int)buf[0];
}

/****************************************************************************
 * Name: esp_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int esp_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS)
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
  int                ret   = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
          }
        else
          {
            /* The USB Serial Console has fixed configuration of:
             *    9600 baudrate, no parity, 8 bits, 1 stopbit.
             */

            termiosp->c_cflag = CS8;
            cfsetispeed(termiosp, 9600);
          }
      }
      break;

    case TCSETS:
      ret = -ENOTTY;
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_usbserial_connected
 *
 * Description:
 *   Poll USB SOF state without writing data or changing the driver's open
 *   lifecycle.  A late-opening console can use this to refresh its
 *   SERIAL_REMOVABLE state before calling open().
 *
 ****************************************************************************/

bool esp_usbserial_connected(void)
{
  return esp_connected(&g_usbserial_priv);
}

/****************************************************************************
 * Name: esp_usbserial_write
 *
 * Description:
 *   Write one character through the USB serial. Used mainly for early
 *   debugging.
 *
 ****************************************************************************/

void esp_usbserial_write(char ch)
{
  uint8_t byte = (uint8_t)ch;

  /* Low-level logging is best-effort but never blocking.  SOF monitoring
   * prevents committing a packet when no host is present; FIFO readiness
   * prevents waiting when a connected host is temporarily behind.
   */

  if (esp_connected(&g_usbserial_priv) &&
      usb_serial_jtag_ll_txfifo_writable())
    {
      usb_serial_jtag_ll_write_txfifo(&byte, 1);
      usb_serial_jtag_ll_txfifo_flush();
    }
}

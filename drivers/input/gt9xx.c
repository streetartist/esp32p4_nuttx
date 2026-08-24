/****************************************************************************
 * drivers/input/gt9xx.c
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

/* Reference:
 * "NuttX RTOS for PinePhone: Touch Panel"
 * https://lupyuen.github.io/articles/touch2
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <poll.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/spinlock.h>
#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/gt9xx.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Default I2C Frequency is 400 kHz */

#ifndef CONFIG_INPUT_GT9XX_I2C_FREQUENCY
#  define CONFIG_INPUT_GT9XX_I2C_FREQUENCY 400000
#endif

#ifndef CONFIG_INPUT_GT9XX_I2C_ADDR
#  define CONFIG_INPUT_GT9XX_I2C_ADDR 0x5d
#endif

#define GT9XX_ADDR_ALT(a)  (((a) == 0x5d) ? 0x14 : 0x5d)

/* Default Number of Poll Waiters is 1 */

#ifndef CONFIG_INPUT_GT9XX_NPOLLWAITERS
#  define CONFIG_INPUT_GT9XX_NPOLLWAITERS 1
#endif

/* I2C Registers for Goodix GT9XX Touch Panel */

#define GTP_REG_VERSION    0x8140  /* Product ID */
#define GTP_READ_COOR_ADDR 0x814e  /* Touch Panel Status */
#define GTP_POINT1         0x8150  /* Touch Point 1 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Touch Panel Device */

struct gt9xx_dev_s
{
  /* I2C bus and address for device */

  FAR struct i2c_master_s *i2c;
  uint8_t addr;

  /* Callback for Board-Specific Operations */

  FAR const struct gt9xx_board_s *board;

  /* Device State */

  mutex_t devlock;  /* Mutex to prevent concurrent reads */
  uint8_t cref;     /* Reference Counter for device */
  bool int_pending; /* True if a Touch Interrupt is pending processing */
  uint16_t x;       /* X Coordinate of Last Touch Point */
  uint16_t y;       /* Y Coordinate of Last Touch Point */
  uint8_t flags;    /* Touch Up or Touch Down for Last Touch Point */

  /* Poll Waiters for device */

  FAR struct pollfd *fds[CONFIG_INPUT_GT9XX_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gt9xx_open(FAR struct file *filep);
static int gt9xx_close(FAR struct file *filep);
static ssize_t gt9xx_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
#ifdef TSIOC_GETMAXPOINTS
static int gt9xx_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#endif
static int gt9xx_poll(FAR struct file *filep, FAR struct pollfd *fds,
                      bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* File Operations for Touch Panel */

static const struct file_operations g_gt9xx_fileops =
{
  gt9xx_open,   /* open */
  gt9xx_close,  /* close */
  gt9xx_read,   /* read */
  NULL,         /* write */
  NULL,         /* seek */
#ifdef TSIOC_GETMAXPOINTS
  gt9xx_ioctl,  /* ioctl */
#else
  NULL,         /* ioctl */
#endif
  NULL,         /* truncate */
  NULL,         /* mmap */
  gt9xx_poll,   /* poll */
  NULL,         /* readv */
  NULL          /* writev */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL        /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gt9xx_i2c_read
 *
 * Description:
 *   Read a Touch Panel Register over I2C.
 *
 * Input Parameters:
 *   dev    - Touch Panel Device
 *   reg    - I2C Register to be read
 *   buf    - Receive Buffer
 *   buflen - Number of bytes to be read
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_i2c_read(FAR struct gt9xx_dev_s *dev,
                          uint16_t reg,
                          uint8_t *buf,
                          size_t buflen)
{
  int ret;

  /* Send the Register Address, MSB first */

  uint8_t regbuf[2] =
  {
    reg >> 8,   /* First Byte: MSB */
    reg & 0xff  /* Second Byte: LSB */
  };

  /* Compose the I2C Messages */

  struct i2c_msg_s msgv[2] =
  {
    {
      /* Send the I2C Register Address */

      .frequency = CONFIG_INPUT_GT9XX_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = 0,
      .buffer    = regbuf,
      .length    = sizeof(regbuf)
    },
    {
      /* Receive the I2C Register Values */

      .frequency = CONFIG_INPUT_GT9XX_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = I2C_M_READ,
      .buffer    = buf,
      .length    = buflen
    }
  };

  const int msgv_len = sizeof(msgv) / sizeof(msgv[0]);

  iinfo("reg=0x%x, buflen=%zu\n", reg, buflen);
  DEBUGASSERT(dev && dev->i2c && buf);

  /* Execute the I2C Transfer.  The first access after reset is often
   * NACKed on this panel; retry before giving up.
   */

    {
      int tries;

      ret = -EIO;
      for (tries = 0; tries < 4; tries++)
        {
          ret = I2C_TRANSFER(dev->i2c, msgv, msgv_len);
          if (ret >= 0)
            {
              break;
            }

          nxsig_usleep(5000);
        }
    }

  if (ret < 0)
    {
      ierr("I2C Read failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_DEBUG_INPUT_INFO
  iinfodumpbuffer("gt9xx_i2c_read", buf, buflen);
#endif /* CONFIG_DEBUG_INPUT_INFO */

  return OK;
}

/****************************************************************************
 * Name: gt9xx_i2c_write
 *
 * Description:
 *   Write to a Touch Panel Register over I2C.
 *
 * Input Parameters:
 *   dev - Touch Panel Device
 *   reg - I2C Register to be written
 *   val - Value to be written
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_i2c_write(FAR struct gt9xx_dev_s *dev,
                           uint16_t reg,
                           uint8_t val)
{
  int ret;

  /* Send the Register Address (MSB first) immediately followed by the
   * Register Value in a single I2C message.  Splitting them with an
   * I2C_M_NOSTART continuation is not supported by every I2C master.
   */

  uint8_t buf[3] =
  {
    reg >> 8,   /* First Byte: Register MSB */
    reg & 0xff, /* Second Byte: Register LSB */
    val         /* Third Byte: Value to be written */
  };

  /* Compose the I2C Message */

  struct i2c_msg_s msgv[1] =
  {
    {
      /* Send the I2C Register Address and Value */

      .frequency = CONFIG_INPUT_GT9XX_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = 0,
      .buffer    = buf,
      .length    = sizeof(buf)
    }
  };

  const int msgv_len = sizeof(msgv) / sizeof(msgv[0]);

  iinfo("reg=0x%x, val=%d\n", reg, val);
  DEBUGASSERT(dev && dev->i2c);

  /* Never retry a status-clear write in the same scan window.  GT911 can
   * NACK one such write transiently; an immediate retry burst can leave its
   * write path wedged until power is removed.
   */

  ret = I2C_TRANSFER(dev->i2c, msgv, msgv_len);

  if (ret < 0)
    {
      ierr("I2C Write failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: gt9xx_probe_device
 *
 * Description:
 *   Read the Product ID from the Touch Panel over I2C.
 *
 * Input Parameters:
 *   dev - Touch Panel Device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_probe_device(FAR struct gt9xx_dev_s *dev)
{
  int ret;
  uint8_t id[4];

  /* Read the Product ID */

  ret = gt9xx_i2c_read(dev, GTP_REG_VERSION, id, sizeof(id));
  if (ret < 0)
    {
      ierr("I2C Probe failed: %d\n", ret);
      return ret;
    }

  /* For GT917S: Product ID will be 39 31 37 53, i.e. "917S" */

#ifdef CONFIG_DEBUG_INPUT_INFO
  iinfodumpbuffer("gt9xx_probe_device", id, sizeof(id));
#endif /* CONFIG_DEBUG_INPUT_INFO */

  return OK;
}

/****************************************************************************
 * Name: gt9xx_set_status
 *
 * Description:
 *   Set the Touch Panel Status over I2C.
 *
 * Input Parameters:
 *   dev    - Touch Panel Device
 *   status - Status value to be set
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_set_status(FAR struct gt9xx_dev_s *dev, uint8_t status)
{
  int ret;

  iinfo("status=%d\n", status);
  DEBUGASSERT(dev);

  /* Write to the Status Register over I2C */

  ret = gt9xx_i2c_write(dev, GTP_READ_COOR_ADDR, status);
  if (ret < 0)
    {
      ierr("Set Status failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: gt9xx_read_touch_data
 *
 * Description:
 *   Read a Touch Sample from Touch Panel. Returns either 0 or 1
 *   Touch Points.
 *
 * Input Parameters:
 *   dev    - Touch Panel Device
 *   sample - Returned Touch Sample (0 or 1 Touch Points)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_read_touch_data(FAR struct gt9xx_dev_s *dev,
                                 FAR struct touch_sample_s *sample)
{
  uint8_t status[1];
  uint8_t touch[5 * 8];
  uint8_t status_code;
  uint8_t touched_points;
  uint16_t x = 0;
  uint16_t y = 0;
  uint8_t flags;
  int ret;

  /* Erase the Touch Sample and Touch Point */

  iinfo("\n");
  DEBUGASSERT(dev && sample);
  memset(sample, 0, sizeof(*sample));

  /* Official esp_lcd_touch_gt911: 1-byte 0x814E, then 8 bytes from
   * 0x814F (track, xL, xH, yL, yH, sizeL, sizeH, reserved).  A single
   * 8-byte burst starting at 0x814E NACKs on this I2C master.
   */

  ret = gt9xx_i2c_read(dev, GTP_READ_COOR_ADDR, status, sizeof(status));
  if (ret < 0)
    {
      ierr("Read Touch Panel Status failed: %d\n", ret);
      return ret;
    }

  status_code = status[0] & 0x80;
  touched_points = status[0] & 0x0f;

  if (status_code == 0)
    {
      /* The ready bit only says whether a new coordinate frame is waiting.
       * It is normally clear while a stationary finger remains down after
       * the previous frame was acknowledged.  Preserve the tracked contact
       * until the controller publishes a ready frame with zero points.
       */

      return -EAGAIN;
    }

  if (touched_points == 0 || touched_points > 5)
    {
      gt9xx_set_status(dev, 0);
      return OK;
    }

  /* The official driver reads all reported point records in one transfer,
   * beginning at 0x814f, before clearing 0x814e.
   */

  ret = gt9xx_i2c_read(dev, GTP_READ_COOR_ADDR + 1, touch,
                       touched_points * 8);
  if (ret < 0)
    {
      ierr("Read Touch Point failed: %d\n", ret);
      return ret;
    }

  ret = gt9xx_set_status(dev, 0);
  if (ret < 0)
    {
      return ret;
    }

  x = touch[1] + (touch[2] << 8);
  y = touch[3] + (touch[4] << 8);
  flags = TOUCH_DOWN | TOUCH_ID_VALID | TOUCH_POS_VALID;
  sample->npoints = 1;
  sample->point[0].id = touch[0];
  sample->point[0].x = x;
  sample->point[0].y = y;
  sample->point[0].flags = flags;

  return OK;
}

/****************************************************************************
 * Name: gt9xx_read
 *
 * Description:
 *   Read a Touch Sample from Touch Panel. Returns either 0 or 1
 *   Touch Points.
 *
 * Input Parameters:
 *   dev    - Touch Panel Device
 *   buffer - Returned Touch Sample (0 or 1 Touch Points)
 *   buflen - Size of buffer
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: gt9xx_ioctl
 *
 * Description:
 *   Handle Touch Panel ioctl commands.  Currently only
 *   TSIOC_GETMAXPOINTS is supported, which reports the maximum number of
 *   simultaneous touch points of the GT9xx family.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

#ifdef TSIOC_GETMAXPOINTS
static int gt9xx_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  switch (cmd)
    {
      case TSIOC_GETMAXPOINTS:
        {
          FAR uint8_t *maxpoints = (FAR uint8_t *)((uintptr_t)arg);

          if (maxpoints == NULL)
            {
              return -EINVAL;
            }

          /* This driver reports a single touch point per sample (readers
           * size their buffers from this value, so it must describe the
           * sample format rather than the silicon capability).
           */

          *maxpoints = 1;
          return OK;
        }

      default:
        return -ENOTTY;
    }
}
#endif

static ssize_t gt9xx_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode;
  FAR struct gt9xx_dev_s *priv;
  struct touch_sample_s sample;
  const size_t outlen = sizeof(sample);
  irqstate_t flags;
  int ret;

  /* Returned Touch Sample will have 0 or 1 Touch Points */

  iinfo("buflen=%zu\n", buflen);
  if (buflen < outlen)
    {
      ierr("Buffer should be at least %zu bytes, got %zu bytes\n",
           outlen, buflen);
      return -EINVAL;
    }

  /* Get the Touch Panel Device */

  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  /* Begin Mutex: Lock to prevent concurrent reads */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EINVAL;

  /* Contact tracking: TOUCH_DOWN or TOUCH_MOVE in the last reported
   * flags means the finger is still on the glass.
   */

  bool contact = (priv->flags & (TOUCH_DOWN | TOUCH_MOVE)) != 0;

  /* LVGL opens O_NONBLOCK and polls by calling read().  Do not require
   * int_pending; the vendor BSP also reads 0x814e periodically.
   */

  /* Read the Touch Report over I2C */

  ret = gt9xx_read_touch_data(priv, &sample);

  /* Begin Critical Section: clear the Interrupt Pending Flag */

  flags = enter_critical_section();
  priv->int_pending = false;
  leave_critical_section(flags);

  if (ret == -EAGAIN)
    {
      /* The controller has no new report: keep the current state */

      if ((filep->f_oflags & O_NONBLOCK) != 0)
        {
          nxmutex_unlock(&priv->devlock);
          return -EAGAIN;
        }

      /* Blocking readers get an immediately returned empty sample,
       * matching the historical behavior of this driver: they are
       * expected to use poll() to wait for new data instead of
       * spinning on read().
       */

      memset(&sample, 0, sizeof(sample));
      memcpy(buffer, &sample, sizeof(sample));
      ret = OK;
    }
  else if (ret < 0)
    {
      nxmutex_unlock(&priv->devlock);
      return ret;
    }
  else if (sample.npoints >= 1)
    {
      /* Mirror the coordinates when the panel origin is opposite to
       * the display origin.
       */

#if CONFIG_INPUT_GT9XX_X_INVERT_MAX > 0
      sample.point[0].x = CONFIG_INPUT_GT9XX_X_INVERT_MAX - 1 -
                          sample.point[0].x;
#endif
#if CONFIG_INPUT_GT9XX_Y_INVERT_MAX > 0
      sample.point[0].y = CONFIG_INPUT_GT9XX_Y_INVERT_MAX - 1 -
                          sample.point[0].y;
#endif

      /* Finger on the glass: the first report is Touch Down, any
       * further report while in contact is Touch Move.
       */

      sample.point[0].flags = contact ?
          (TOUCH_MOVE | TOUCH_ID_VALID | TOUCH_POS_VALID) :
          (TOUCH_DOWN | TOUCH_ID_VALID | TOUCH_POS_VALID);
      priv->x = sample.point[0].x;
      priv->y = sample.point[0].y;
      priv->flags = sample.point[0].flags;
      memcpy(buffer, &sample, sizeof(sample));
      iinfo("touch %s x=%d, y=%d\n", contact ? "move" : "down",
            priv->x, priv->y);
    }
  else
    {
      /* Zero-point report from the controller: the finger was lifted */

      if (contact)
        {
          priv->flags = TOUCH_UP | TOUCH_ID_VALID | TOUCH_POS_VALID;
          memset(&sample, 0, sizeof(sample));
          sample.npoints = 1;
          sample.point[0].id = 0;
          sample.point[0].x = priv->x;
          sample.point[0].y = priv->y;
          sample.point[0].flags = priv->flags;
          memcpy(buffer, &sample, sizeof(sample));
          iinfo("touch up x=%d, y=%d\n", priv->x, priv->y);
        }
      else if ((filep->f_oflags & O_NONBLOCK) != 0)
        {
          nxmutex_unlock(&priv->devlock);
          return -EAGAIN;
        }
      else
        {
          memcpy(buffer, &sample, sizeof(sample));
        }
    }

  /* End Mutex: Unlock to allow next read */

  nxmutex_unlock(&priv->devlock);
  return (ret < 0) ? ret : outlen;
}

/****************************************************************************
 * Name: gt9xx_open
 *
 * Description:
 *   Open the Touch Panel Device.  If this is the first open, we power on
 *   the Touch Panel, probe for the Touch Panel and enable Touch Panel
 *   Interrupts.
 *
 * Input Parameters:
 *   filep - File Struct for Touch Panel
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct gt9xx_dev_s *priv;
  unsigned int use_count;
  int ret;

  /* Get the Touch Panel Device */

  iinfo("\n");
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  /* Begin Mutex: Lock to prevent concurrent update to Reference Count */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      ierr("Lock Mutex failed: %d\n", ret);
      return ret;
    }

  /* Get next Reference Count */

  use_count = priv->cref + 1;
  DEBUGASSERT(use_count < UINT8_MAX && use_count > priv->cref);
  if (use_count == 1)
    {
      /* If first user, power on the Touch Panel */

      DEBUGASSERT(priv->board->set_power != NULL);
      ret = priv->board->set_power(priv->board, true);
      if (ret < 0)
        {
          goto out_lock;
        }

      /* Let Touch Panel power up before probing */

      nxsig_usleep(100 * 1000);

      /* Check that Touch Panel exists on I2C */

        {
          int tries;

          ret = -ENODEV;
          for (tries = 0; tries < 5; tries++)
            {
              ret = gt9xx_probe_device(priv);
              if (ret == OK)
                {
                  break;
                }

              if (tries == 2)
                {
                  priv->addr = GT9XX_ADDR_ALT(priv->addr);
                }

              nxsig_usleep(50 * 1000);
            }
        }

      if (ret < 0)
        {
          /* No such device, power off the Touch Panel */

          priv->board->set_power(priv->board, false);
          goto out_lock;
        }

      gt9xx_set_status(priv, 0);
      ret = OK;

      /* Enable Touch Panel Interrupts */

      DEBUGASSERT(priv->board->irq_enable);
      priv->board->irq_enable(priv->board, true);
    }

  /* Set the Reference Count */

  priv->cref = use_count;

  /* End Mutex: Unlock to allow update to Reference Count */

out_lock:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: gt9xx_close
 *
 * Description:
 *   Close the Touch Panel Device.  If this is the final close, we disable
 *   Touch Panel Interrupts and power off the Touch Panel.
 *
 * Input Parameters:
 *   filep - File Struct for Touch Panel
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct gt9xx_dev_s *priv;
  int use_count;
  int ret;

  /* Get the Touch Panel Device */

  iinfo("\n");
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  /* Begin Mutex: Lock to prevent concurrent update to Reference Count */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      ierr("Lock Mutex failed: %d\n", ret);
      return ret;
    }

  /* Decrement the Reference Count */

  use_count = priv->cref - 1;
  DEBUGASSERT(use_count >= 0);
  if (use_count == 0)
    {
      /* If final user, disable Touch Panel Interrupts */

      DEBUGASSERT(priv->board && priv->board->irq_enable);
      priv->board->irq_enable(priv->board, false);

      /* Power off the Touch Panel */

      DEBUGASSERT(priv->board->set_power);
      priv->board->set_power(priv->board, false);
    }

  /* Set the Reference Count */

  priv->cref = use_count;

  /* End Mutex: Unlock to allow update to Reference Count */

  nxmutex_unlock(&priv->devlock);
  return OK;
}

/****************************************************************************
 * Name: gt9xx_poll
 *
 * Description:
 *   Setup or teardown a poll for the Touch Panel Device.
 *
 * Input Parameters:
 *   filep - File Struct for Touch Panel
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *   setup - true: Setup the poll; false: Teardown the poll
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_poll(FAR struct file *filep, FAR struct pollfd *fds,
                      bool setup)
{
  FAR struct gt9xx_dev_s *priv;
  FAR struct inode *inode;
  bool pending;
  int ret = 0;
  int i;

  /* Get the Touch Panel Device */

  iinfo("setup=%d\n", setup);
  DEBUGASSERT(fds);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  /* Begin Mutex: Lock to prevent concurrent update to Poll Waiters */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      ierr("Lock Mutex failed: %d\n", ret);
      return ret;
    }

  if (setup)
    {
      /* If Poll Setup: Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto out;
        }

      /* Find an available slot for the Poll Waiter */

      for (i = 0; i < CONFIG_INPUT_GT9XX_NPOLLWAITERS; i++)
        {
          /* Found an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_INPUT_GT9XX_NPOLLWAITERS)
        {
          /* No slots available */

          fds->priv = NULL;
          ret = -EBUSY;
        }
      else
        {
          /* If Interrupt Pending is set, notify the Poll Waiters */

          pending = priv->int_pending;
          if (pending)
            {
              poll_notify(&fds, 1, POLLIN);
            }
        }
    }
  else if (fds->priv)
    {
      /* If Poll Teardown: Remove the poll setup */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      *slot = NULL;
      fds->priv = NULL;
    }

  /* End Mutex: Unlock to allow update to Poll Waiters */

out:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: gt9xx_isr_handler
 *
 * Description:
 *   Interrupt Handler for Touch Panel.
 *
 * Input Parameters:
 *   irq     - IRQ Number
 *   context - IRQ Context
 *   arg     - Touch Panel Device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_isr_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct gt9xx_dev_s *priv = (FAR struct gt9xx_dev_s *)arg;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* Begin Critical Section */

  flags = enter_critical_section();

  /* Set the Interrupt Pending Flag */

  priv->int_pending = true;

  /* End Critical Section */

  leave_critical_section(flags);

  /* Notify the Poll Waiters */

  poll_notify(priv->fds, CONFIG_INPUT_GT9XX_NPOLLWAITERS, POLLIN);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gt9xx_register
 *
 * Description:
 *   Register the driver for Goodix GT9XX Touch Panel.  Attach the
 *   Interrupt Handler for the Touch Panel and disable Touch Interrupts.
 *
 * Input Parameters:
 *   devpath      - Device Path (e.g. "/dev/input0")
 *   dev          - I2C Bus
 *   i2c_devaddr  - I2C Address of Touch Panel
 *   board_config - Callback for Board-Specific Operations
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int gt9xx_register(FAR const char *devpath,
                   FAR struct i2c_master_s *i2c_dev,
                   uint8_t i2c_devaddr,
                   const struct gt9xx_board_s *board_config)
{
  struct gt9xx_dev_s *priv;
  int ret = 0;

  iinfo("devpath=%s, i2c_devaddr=%d\n", devpath, i2c_devaddr);
  DEBUGASSERT(devpath != NULL && i2c_dev != NULL && board_config != NULL);

  /* Allocate the Touch Panel Device Structure */

  priv = kmm_zalloc(sizeof(struct gt9xx_dev_s));
  if (!priv)
    {
      ierr("GT9XX Memory Allocation failed\n");
      return -ENOMEM;
    }

  /* Setup the Touch Panel Device Structure */

  priv->addr = i2c_devaddr;
  priv->i2c = i2c_dev;
  priv->board = board_config;
  nxmutex_init(&priv->devlock);

  /* Register the Touch Input Driver */

  ret = register_driver(devpath, &g_gt9xx_fileops, 0666, priv);
  if (ret < 0)
    {
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      ierr("GT9XX Registration failed: %d\n", ret);
      return ret;
    }

  /* Attach the Interrupt Handler */

  DEBUGASSERT(priv->board->irq_attach);
  priv->board->irq_attach(priv->board, gt9xx_isr_handler, priv);

  /* Disable Touch Panel Interrupts */

  DEBUGASSERT(priv->board->irq_enable);
  priv->board->irq_enable(priv->board, false);

  iinfo("GT9XX Touch Panel registered\n");
  return OK;
}

/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4_lcd.c
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

#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/video/fb.h>
#include <nuttx/video/mipi_dsi.h>

#include "espressif/esp_gpio.h"
#include "espressif/esp_ldo.h"
#include "espressif/esp_mipi_dsi.h"

#include "esp32p4-function-ev-board.h"

#ifdef CONFIG_ESP32P4_FUNCTION_EV_BOARD_LCD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ESP32-P4-Function-EV-Board 7-inch 1024x600 EK79007 panel. */

#define LCD_XRES                   1024
#define LCD_YRES                   600
#define LCD_BPP                    16
#define LCD_STRIDE                 (LCD_XRES * LCD_BPP / 8)
#define LCD_FB_SIZE                (LCD_STRIDE * LCD_YRES)

#define LCD_GPIO_RST               27
#define LCD_GPIO_BL                26

#define LCD_DSI_LANES              2
#define LCD_DSI_RATE_MBPS          1000
#define LCD_DPHY_LDO_CHAN          3
#define LCD_DPHY_LDO_MV            2500

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ek79007_cmd_s
{
  uint8_t cmd;
  uint8_t data;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int lcd_getvideoinfo(FAR struct fb_vtable_s *vtable,
                            FAR struct fb_videoinfo_s *vinfo);
static int lcd_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                            FAR struct fb_planeinfo_s *pinfo);
#ifdef CONFIG_FB_UPDATE
static int lcd_updatearea(FAR struct fb_vtable_s *vtable,
                          FAR const struct fb_area_s *area);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Vendor sequence used by Espressif's EK79007 BSP. Sleep-out and display-on
 * are issued separately after the framebuffer has been bound.
 */

static const struct ek79007_cmd_s g_ek79007_init[] =
{
  {0xb2, 0x10},             /* Pad control: 2-lane MIPI */
  {0x80, 0x8b},
  {0x81, 0x78},
  {0x82, 0x84},
  {0x83, 0x88},
  {0x84, 0xa8},
  {0x85, 0xe3},
  {0x86, 0x88},
};

static const uint16_t g_rainbow[] =
{
  0xf800,                   /* Red */
  0xfd20,                   /* Orange */
  0xffe0,                   /* Yellow */
  0x07e0,                   /* Green */
  0x07ff,                   /* Cyan */
  0x001f,                   /* Blue */
  0xf81f,                   /* Magenta */
};

static struct fb_vtable_s g_lcd_vtable =
{
  .getvideoinfo = lcd_getvideoinfo,
  .getplaneinfo = lcd_getplaneinfo,
#ifdef CONFIG_FB_UPDATE
  .updatearea   = lcd_updatearea,
#endif
};

static struct fb_videoinfo_s g_lcd_video =
{
  .fmt     = FB_FMT_RGB16_565,
  .xres    = LCD_XRES,
  .yres    = LCD_YRES,
  .nplanes = 1,
};

static struct fb_planeinfo_s g_lcd_plane =
{
  .fbmem        = NULL,
  .fblen        = LCD_FB_SIZE,
  .stride       = LCD_STRIDE,
  .display      = 0,
  .bpp          = LCD_BPP,
  .xres_virtual = LCD_XRES,
  .yres_virtual = LCD_YRES,
  .xoffset      = 0,
  .yoffset      = 0,
};

static struct esp_ldo_config_t g_mipi_ldo =
{
  .chan_id    = LCD_DPHY_LDO_CHAN,
  .voltage_mv = LCD_DPHY_LDO_MV,
  .handler    = NULL,
};

static FAR uint16_t *g_lcd_fb;
static bool g_lcd_ready;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int lcd_getvideoinfo(FAR struct fb_vtable_s *vtable,
                            FAR struct fb_videoinfo_s *vinfo)
{
  if (vtable != &g_lcd_vtable || vinfo == NULL)
    {
      return -EINVAL;
    }

  memcpy(vinfo, &g_lcd_video, sizeof(*vinfo));
  return OK;
}

static int lcd_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                            FAR struct fb_planeinfo_s *pinfo)
{
  if (vtable != &g_lcd_vtable || pinfo == NULL || planeno != 0 ||
      g_lcd_plane.fbmem == NULL)
    {
      return -EINVAL;
    }

  memcpy(pinfo, &g_lcd_plane, sizeof(*pinfo));
  return OK;
}

#ifdef CONFIG_FB_UPDATE
static int lcd_updatearea(FAR struct fb_vtable_s *vtable,
                          FAR const struct fb_area_s *area)
{
  size_t offset;
  size_t len;

  if (vtable != &g_lcd_vtable || g_lcd_fb == NULL)
    {
      return -EAGAIN;
    }

  if (area == NULL)
    {
      return esp_mipi_dsi_flush_framebuffer(g_lcd_fb, LCD_FB_SIZE);
    }

  if (area->x >= LCD_XRES || area->y >= LCD_YRES)
    {
      return OK;
    }

  offset = (size_t)area->y * LCD_STRIDE +
           (size_t)area->x * (LCD_BPP / 8);
  len = (size_t)area->h * LCD_STRIDE;
  if (offset + len > LCD_FB_SIZE)
    {
      len = LCD_FB_SIZE - offset;
    }

  return esp_mipi_dsi_flush_framebuffer(
      (FAR uint8_t *)g_lcd_fb + offset, len);
}
#endif

static int lcd_panel_initialize(FAR struct mipi_dsi_host *host,
                                FAR struct mipi_dsi_device **panel)
{
  FAR struct mipi_dsi_device *device;
  size_t i;
  int ret;

  device = mipi_dsi_device_register(host, "ek79007", 0);
  if (device == NULL)
    {
      return -ENODEV;
    }

  device->lanes      = LCD_DSI_LANES;
  device->format     = MIPI_DSI_FMT_RGB565;
  device->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
                       MIPI_DSI_MODE_LPM;
  device->hs_rate    = LCD_DSI_RATE_MBPS * 1000000UL;
  device->lp_rate    = 0;

  ret = mipi_dsi_attach(device);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < sizeof(g_ek79007_init) / sizeof(g_ek79007_init[0]); i++)
    {
      ret = mipi_dsi_dcs_write(device, g_ek79007_init[i].cmd,
                               &g_ek79007_init[i].data, 1);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: EK79007 DCS 0x%02x failed: %d\n",
                 g_ek79007_init[i].cmd, ret);
          return ret;
        }
    }

  *panel = device;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int esp32p4_lcd_show_rainbow(void)
{
  size_t colors = sizeof(g_rainbow) / sizeof(g_rainbow[0]);
  size_t x;
  size_t y;

  if (!g_lcd_ready || g_lcd_fb == NULL)
    {
      return -EAGAIN;
    }

  for (y = 0; y < LCD_YRES; y++)
    {
      for (x = 0; x < LCD_XRES; x++)
        {
          g_lcd_fb[y * LCD_XRES + x] =
              g_rainbow[(x * colors) / LCD_XRES];
        }
    }

  return esp_mipi_dsi_flush_framebuffer(g_lcd_fb, LCD_FB_SIZE);
}

int up_fbinitialize(int display)
{
  FAR struct mipi_dsi_host *host;
  FAR struct mipi_dsi_device *panel;
  struct esp_mipi_dsi_bus_config_s bus =
  {
    .num_data_lanes     = LCD_DSI_LANES,
    .lane_bit_rate_mbps = LCD_DSI_RATE_MBPS,
  };
  struct esp_mipi_dsi_dpi_config_s dpi =
  {
    .h_res                 = LCD_XRES,
    .v_res                 = LCD_YRES,
    .hsync_pulse_width     = 10,
    .hsync_back_porch      = 160,
    .hsync_front_porch     = 160,
    .vsync_pulse_width     = 1,
    .vsync_back_porch      = 23,
    .vsync_front_porch     = 12,
    .dpi_clock_freq_mhz    = 52,
    .virtual_channel       = 0,
    .format                = MIPI_DSI_FMT_RGB565,
  };
  int ret;

  if (display != 0)
    {
      return -ENODEV;
    }

  if (g_lcd_ready)
    {
      return OK;
    }

  ret = esp_configgpio(LCD_GPIO_BL, OUTPUT);
  if (ret < 0)
    {
      return ret;
    }

  esp_gpiowrite(LCD_GPIO_BL, false);

  ret = esp_ldo_channel_acquire(&g_mipi_ldo);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: MIPI DPHY LDO failed: %d\n", ret);
      return ret;
    }

  ret = esp_configgpio(LCD_GPIO_RST, OUTPUT);
  if (ret < 0)
    {
      return ret;
    }

  esp_gpiowrite(LCD_GPIO_RST, false);
  up_mdelay(10);
  esp_gpiowrite(LCD_GPIO_RST, true);
  up_mdelay(20);

  ret = esp_mipi_dsi_initialize(&bus);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: MIPI-DSI host failed: %d\n", ret);
      return ret;
    }

  host = esp_mipi_dsi_host_get();
  if (host == NULL)
    {
      return -ENODEV;
    }

  ret = esp_mipi_dsi_configure_dpi(&dpi);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: MIPI DPI config failed: %d\n", ret);
      return ret;
    }

  ret = lcd_panel_initialize(host, &panel);
  if (ret < 0)
    {
      return ret;
    }

  /* The PSRAM-backed user heap is large enough for the 1.2 MiB scanout FB.
   * Keep the internal kernel heap available for NuttX objects and stacks.
   */

  g_lcd_fb = memalign(64, LCD_FB_SIZE);
  if (g_lcd_fb == NULL)
    {
      syslog(LOG_ERR, "ERROR: LCD FB alloc failed (%u bytes)\n",
             (unsigned int)LCD_FB_SIZE);
      return -ENOMEM;
    }

  g_lcd_plane.fbmem = g_lcd_fb;

  ret = esp_mipi_dsi_bind_framebuffer(g_lcd_fb, LCD_FB_SIZE,
                                      LCD_XRES, LCD_YRES, LCD_BPP);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: LCD FB bind failed: %d\n", ret);
      goto errout_fb;
    }

  g_lcd_ready = true;
  ret = esp32p4_lcd_show_rainbow();
  if (ret < 0)
    {
      goto errout_fb;
    }

  ret = mipi_dsi_dcs_exit_sleep_mode(panel);
  if (ret < 0)
    {
      goto errout_fb;
    }

  up_mdelay(120);

  ret = esp_mipi_dsi_video_start();
  if (ret < 0)
    {
      goto errout_fb;
    }

  ret = mipi_dsi_dcs_set_display_on(panel);
  if (ret < 0)
    {
      /* DMA owns the framebuffer after video_start; keep it allocated. */

      g_lcd_ready = false;
      return ret;
    }

  up_mdelay(20);
  esp_gpiowrite(LCD_GPIO_BL, true);

  return OK;

errout_fb:
  g_lcd_ready = false;
  g_lcd_plane.fbmem = NULL;
  free(g_lcd_fb);
  g_lcd_fb = NULL;
  return ret;
}

FAR struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  if (display != 0 || vplane != 0 || !g_lcd_ready)
    {
      return NULL;
    }

  return &g_lcd_vtable;
}

void up_fbuninitialize(int display)
{
  UNUSED(display);
}

#endif /* CONFIG_ESP32P4_FUNCTION_EV_BOARD_LCD */

/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4_camera.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ESP32P4_FUNCTION_EV_BOARD_CAMERA

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/imgdata.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/v4l2_cap.h>
#include <nuttx/video/video.h>

#include "espressif/esp_i2c.h"
#include "espressif/esp_ldo.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_csi.h"
#include "driver/isp_core.h"
#include "driver/isp_types.h"
#include "driver/isp_demosaic.h"
#include "driver/isp_ccm.h"
#include "driver/isp_color.h"
#include "hal/color_types.h"
#include "hal/mipi_csi_types.h"
#include "soc/clk_tree_defs.h"

#include "esp32p4-function-ev-board.h"

#define CAMERA_I2C_PORT          0
#define CAMERA_I2C_ADDR          0x30
#define CAMERA_I2C_FREQUENCY     100000
#define CAMERA_WIDTH             1280
#define CAMERA_HEIGHT            720
#define CAMERA_BPP               2
#define CAMERA_FRAME_SIZE        (CAMERA_WIDTH * CAMERA_HEIGHT * CAMERA_BPP)
#define CAMERA_BUF_ALIGN         64
#define CAMERA_LANE_NUM          2
#define CAMERA_LANE_BITRATE      405
#define CAMERA_CSI_ID            0
#define CAMERA_LDO_CHANNEL       3
#define CAMERA_LDO_MV            2500
#define SC2336_PID               0xcb3a
#define SC2336_REG_PID_H         0x3107
#define SC2336_REG_PID_L         0x3108
#define SC2336_REG_FLIP_MIRROR   0x3221
#define SC2336_REG_STREAM        0x0100
#define SC2336_REG_SLEEP_MODE    SC2336_REG_STREAM
#define SC2336_REG_DELAY         0xfffe
#define SC2336_REG_END           0xffff

typedef struct
{
  uint16_t reg;
  uint8_t val;
} sc2336_reginfo_t;

#include "esp32p4_sc2336_regs.h"

struct esp32p4_camera_s
{
  struct imgsensor_s sensor;
  struct imgdata_s data;
  struct i2c_master_s *i2c;
  struct esp_ldo_config_t ldo;
  esp_cam_ctlr_handle_t csi;
  isp_proc_handle_t isp;
  imgdata_capture_t callback;
  FAR void *callback_arg;
  FAR uint8_t *next_buffer;
  uint32_t next_size;
  uint8_t flip_mirror;
  bool streaming;
};

static struct esp32p4_camera_s g_camera;

static void camera_isp_pipeline_disable(FAR struct esp32p4_camera_s *priv);

static int camera_i2c_write(FAR struct esp32p4_camera_s *priv,
                            uint16_t reg, uint8_t value)
{
  uint8_t buffer[3] = { reg >> 8, reg & 0xff, value };
  struct i2c_msg_s msg =
  {
    .frequency = CAMERA_I2C_FREQUENCY,
    .addr      = CAMERA_I2C_ADDR,
    .flags     = 0,
    .buffer    = buffer,
    .length    = sizeof(buffer),
  };

  return I2C_TRANSFER(priv->i2c, &msg, 1) < 0 ? -EIO : 0;
}

static int camera_i2c_read(FAR struct esp32p4_camera_s *priv,
                           uint16_t reg, FAR uint8_t *value)
{
  uint8_t address[2] = { reg >> 8, reg & 0xff };
  struct i2c_msg_s msg[2] =
  {
    {
      .frequency = CAMERA_I2C_FREQUENCY,
      .addr      = CAMERA_I2C_ADDR,
      .flags     = 0,
      .buffer    = address,
      .length    = sizeof(address),
    },
    {
      .frequency = CAMERA_I2C_FREQUENCY,
      .addr      = CAMERA_I2C_ADDR,
      .flags     = I2C_M_READ,
      .buffer    = value,
      .length    = 1,
    }
  };

  return I2C_TRANSFER(priv->i2c, msg, 2) < 0 ? -EIO : 0;
}

static bool camera_is_available(FAR struct imgsensor_s *sensor)
{
  FAR struct esp32p4_camera_s *priv =
    container_of(sensor, struct esp32p4_camera_s, sensor);
  uint8_t pid_h;
  uint8_t pid_l;

  if (priv->i2c == NULL || camera_i2c_read(priv, SC2336_REG_PID_H, &pid_h) < 0 ||
      camera_i2c_read(priv, SC2336_REG_PID_L, &pid_l) < 0)
    {
      return false;
    }

  return ((uint16_t)pid_h << 8 | pid_l) == SC2336_PID;
}

static int camera_sensor_init(FAR struct imgsensor_s *sensor)
{
  FAR struct esp32p4_camera_s *priv =
    container_of(sensor, struct esp32p4_camera_s, sensor);
  size_t i;
  int ret;

  for (i = 0; sc2336_mipi_2lane_24Minput_1280x720_raw10_30fps[i].reg !=
              SC2336_REG_END; i++)
    {
      uint16_t reg = sc2336_mipi_2lane_24Minput_1280x720_raw10_30fps[i].reg;
      uint8_t val = sc2336_mipi_2lane_24Minput_1280x720_raw10_30fps[i].val;

      if (reg == SC2336_REG_DELAY)
        {
          up_mdelay(val);
        }
      else
        {
          ret = camera_i2c_write(priv, reg, val);
          if (ret < 0)
            {
              return ret;
            }

          if (reg == 0x0103)
            {
              up_mdelay(5);
            }
        }
    }

  priv->flip_mirror = 0;
  return 0;
}

static int camera_sensor_uninit(FAR struct imgsensor_s *sensor)
{
  return 0;
}

static const char *camera_get_driver_name(FAR struct imgsensor_s *sensor)
{
  return "SC2336 NuttX MIPI-CSI";
}

static int camera_validate_sensor_format(FAR struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_datafmts,
                                         FAR imgsensor_format_t *datafmts,
                                         FAR imgsensor_interval_t *interval)
{
  if (nr_datafmts < 1 || datafmts == NULL || interval == NULL ||
      datafmts[0].width != CAMERA_WIDTH ||
      datafmts[0].height != CAMERA_HEIGHT ||
      datafmts[0].pixelformat != IMGSENSOR_PIX_FMT_RGB565)
    {
      return -EINVAL;
    }

  if (interval->numerator == 0 || interval->denominator == 0)
    {
      return -EINVAL;
    }

  return 0;
}

static int camera_sensor_start(FAR struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type,
                               uint8_t nr_datafmts,
                               FAR imgsensor_format_t *datafmts,
                               FAR imgsensor_interval_t *interval)
{
  FAR struct esp32p4_camera_s *priv =
    container_of(sensor, struct esp32p4_camera_s, sensor);
  return camera_i2c_write(priv, SC2336_REG_STREAM, 1);
}

static int camera_sensor_stop(FAR struct imgsensor_s *sensor,
                              imgsensor_stream_type_t type)
{
  FAR struct esp32p4_camera_s *priv =
    container_of(sensor, struct esp32p4_camera_s, sensor);
  return camera_i2c_write(priv, SC2336_REG_STREAM, 0);
}

static int camera_get_interval(FAR struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type,
                               FAR imgsensor_interval_t *interval)
{
  if (interval == NULL)
    {
      return -EINVAL;
    }

  interval->numerator = 1;
  interval->denominator = 30;
  return 0;
}

static int camera_get_supported(FAR struct imgsensor_s *sensor, uint32_t id,
                                FAR imgsensor_supported_value_t *value)
{
  if (value == NULL)
    {
      return -EINVAL;
    }

  if (id == IMGSENSOR_ID_HFLIP_VIDEO || id == IMGSENSOR_ID_VFLIP_VIDEO ||
      id == IMGSENSOR_ID_HFLIP_STILL || id == IMGSENSOR_ID_VFLIP_STILL)
    {
      value->type = IMGSENSOR_CTRL_TYPE_BOOLEAN;
      value->u.range.minimum = 0;
      value->u.range.maximum = 1;
      value->u.range.step = 1;
      value->u.range.default_value = 0;
      return 0;
    }

  return -ENOTTY;
}

static int camera_get_value(FAR struct imgsensor_s *sensor, uint32_t id,
                            uint32_t size, FAR imgsensor_value_t *value)
{
  FAR struct esp32p4_camera_s *priv =
    container_of(sensor, struct esp32p4_camera_s, sensor);

  if (value == NULL || size < sizeof(int32_t))
    {
      return -EINVAL;
    }

  if (id == IMGSENSOR_ID_HFLIP_VIDEO || id == IMGSENSOR_ID_HFLIP_STILL)
    {
      value->value32 = (priv->flip_mirror >> 1) & 1;
      return 0;
    }

  if (id == IMGSENSOR_ID_VFLIP_VIDEO || id == IMGSENSOR_ID_VFLIP_STILL)
    {
      value->value32 = (priv->flip_mirror >> 5) & 1;
      return 0;
    }

  return -ENOTTY;
}

static int camera_set_value(FAR struct imgsensor_s *sensor, uint32_t id,
                            uint32_t size, imgsensor_value_t value)
{
  FAR struct esp32p4_camera_s *priv =
    container_of(sensor, struct esp32p4_camera_s, sensor);
  uint8_t regval;
  bool hflip;
  bool vflip;

  if (size < sizeof(int32_t))
    {
      return -EINVAL;
    }

  hflip = (id == IMGSENSOR_ID_HFLIP_VIDEO || id == IMGSENSOR_ID_HFLIP_STILL);
  vflip = (id == IMGSENSOR_ID_VFLIP_VIDEO || id == IMGSENSOR_ID_VFLIP_STILL);
  if (!hflip && !vflip)
    {
      return -ENOTTY;
    }

  if (hflip)
    {
      if (value.value32)
        {
          priv->flip_mirror |= 3 << 1;
        }
      else
        {
          priv->flip_mirror &= ~(3 << 1);
        }
    }

  if (vflip)
    {
      if (value.value32)
        {
          priv->flip_mirror |= 3 << 5;
        }
      else
        {
          priv->flip_mirror &= ~(3 << 5);
        }
    }

  regval = priv->flip_mirror;
  return camera_i2c_write(priv, SC2336_REG_FLIP_MIRROR, regval);
}

static const struct imgsensor_ops_s g_sensor_ops =
{
  .is_available           = camera_is_available,
  .init                   = camera_sensor_init,
  .uninit                 = camera_sensor_uninit,
  .get_driver_name        = camera_get_driver_name,
  .validate_frame_setting = camera_validate_sensor_format,
  .start_capture          = camera_sensor_start,
  .stop_capture           = camera_sensor_stop,
  .get_frame_interval     = camera_get_interval,
  .get_supported_value    = camera_get_supported,
  .get_value              = camera_get_value,
  .set_value              = camera_set_value,
};

static bool camera_on_new_trans(esp_cam_ctlr_handle_t handle,
                                FAR esp_cam_ctlr_trans_t *trans,
                                FAR void *user_data)
{
  FAR struct esp32p4_camera_s *priv = user_data;

  if (priv == NULL || trans == NULL || priv->next_buffer == NULL)
    {
      return false;
    }

  trans->buffer = priv->next_buffer;
  trans->buflen = priv->next_size;
  priv->next_buffer = NULL;
  priv->next_size = 0;
  return true;
}

static bool camera_on_finished(esp_cam_ctlr_handle_t handle,
                               FAR esp_cam_ctlr_trans_t *trans,
                               FAR void *user_data)
{
  FAR struct esp32p4_camera_s *priv = user_data;

  if (priv != NULL && trans != NULL && priv->callback != NULL)
    {
      priv->callback(0, trans->received_size, NULL, priv->callback_arg);
    }

  return true;
}

static int camera_data_init(FAR struct imgdata_s *data)
{
  (void)data;

  /* MIPI DSI already acquired LDO channel 3 for the shared D-PHY.
   * Do not acquire/release it from the camera path.
   */

  return 0;
}

static int camera_data_uninit(FAR struct imgdata_s *data)
{
  FAR struct esp32p4_camera_s *priv =
    container_of(data, struct esp32p4_camera_s, data);

  priv->streaming = false;
  priv->next_buffer = NULL;
  priv->callback = NULL;
  priv->callback_arg = NULL;

  if (priv->csi != NULL)
    {
      (void)esp_cam_ctlr_stop(priv->csi);
      (void)esp_cam_ctlr_disable(priv->csi);
      (void)esp_cam_ctlr_del(priv->csi);
      priv->csi = NULL;
    }

  if (priv->isp != NULL)
    {
      camera_isp_pipeline_disable(priv);
      (void)esp_isp_disable(priv->isp);
      (void)esp_isp_del_processor(priv->isp);
      priv->isp = NULL;
    }

  return 0;
}

static int camera_data_set_buf(FAR struct imgdata_s *data,
                               uint8_t nr_datafmts,
                               FAR imgdata_format_t *datafmts,
                               FAR uint8_t *addr, uint32_t size)
{
  FAR struct esp32p4_camera_s *priv =
    container_of(data, struct esp32p4_camera_s, data);

  if (nr_datafmts < 1 || datafmts == NULL || addr == NULL ||
      size < CAMERA_FRAME_SIZE ||
      ((uintptr_t)addr & (CAMERA_BUF_ALIGN - 1)) != 0)
    {
      printf("Camera: set_buf reject addr=%p size=%u align=%u\n",
             addr, (unsigned int)size,
             (unsigned int)((uintptr_t)addr & (CAMERA_BUF_ALIGN - 1)));
      return -EINVAL;
    }

  priv->next_buffer = addr;
  priv->next_size = size;
  return 0;
}

static int camera_isp_pipeline_enable(FAR struct esp32p4_camera_s *priv)
{
  esp_isp_demosaic_config_t demosaic;
  esp_isp_ccm_config_t ccm;
  esp_isp_color_config_t color;
  int ret;

  memset(&demosaic, 0, sizeof(demosaic));
  demosaic.grad_ratio.integer = 1;
  ret = esp_isp_demosaic_configure(priv->isp, &demosaic);
  if (ret == ESP_OK)
    {
      ret = esp_isp_demosaic_enable(priv->isp);
    }

  if (ret != ESP_OK)
    {
      printf("Camera: demosaic failed: %d\n", ret);
      return ret;
    }

  /* RAW sensors look green without WB.  P4 v1 has no WBG block, so
   * apply red/blue gains in CCM like esp_video does on older silicon.
   */
  memset(&ccm, 0, sizeof(ccm));
  ccm.saturation = true;
  ccm.matrix[0][0] = 1.70f;
  ccm.matrix[1][1] = 0.95f;
  ccm.matrix[2][2] = 1.55f;
  ret = esp_isp_ccm_configure(priv->isp, &ccm);
  if (ret == ESP_OK)
    {
      ret = esp_isp_ccm_enable(priv->isp);
    }

  if (ret != ESP_OK)
    {
      printf("Camera: ccm failed: %d\n", ret);
      return ret;
    }

  memset(&color, 0, sizeof(color));
  color.color_contrast.val = 128;
  color.color_saturation.val = 128;
  color.color_hue = 0;
  color.color_brightness = 0;
  ret = esp_isp_color_configure(priv->isp, &color);
  if (ret == ESP_OK)
    {
      ret = esp_isp_color_enable(priv->isp);
    }

  if (ret != ESP_OK)
    {
      printf("Camera: color failed: %d\n", ret);
    }

  return ret;
}

static void camera_isp_pipeline_disable(FAR struct esp32p4_camera_s *priv)
{
  if (priv->isp == NULL)
    {
      return;
    }

  (void)esp_isp_color_disable(priv->isp);
  (void)esp_isp_ccm_disable(priv->isp);
  (void)esp_isp_demosaic_disable(priv->isp);
}

static int camera_data_validate(FAR struct imgdata_s *data,
                                uint8_t nr_datafmts,
                                FAR imgdata_format_t *datafmts,
                                FAR imgdata_interval_t *interval)
{
  if (nr_datafmts < 1 || datafmts == NULL || interval == NULL ||
      datafmts[0].width != CAMERA_WIDTH ||
      datafmts[0].height != CAMERA_HEIGHT ||
      datafmts[0].pixelformat != IMGDATA_PIX_FMT_RGB565)
    {
      printf("Camera: validate fail nr=%u %ux%u fmt=%u\n",
             (unsigned int)nr_datafmts,
             datafmts ? datafmts[0].width : 0,
             datafmts ? datafmts[0].height : 0,
             datafmts ? (unsigned int)datafmts[0].pixelformat : 0);
      return -EINVAL;
    }

  return 0;
}

static int camera_data_start(FAR struct imgdata_s *data,
                             uint8_t nr_datafmts,
                             FAR imgdata_format_t *datafmts,
                             FAR imgdata_interval_t *interval,
                             imgdata_capture_t callback, FAR void *arg)
{
  FAR struct esp32p4_camera_s *priv =
    container_of(data, struct esp32p4_camera_s, data);
  esp_cam_ctlr_csi_config_t csi_config;
  esp_cam_ctlr_evt_cbs_t callbacks;
  esp_isp_processor_cfg_t isp_config;
  int ret;

  if (camera_data_validate(data, nr_datafmts, datafmts, interval) < 0 ||
      callback == NULL || priv->next_buffer == NULL)
    {
      printf("Camera: start reject cb=%p buf=%p\n",
             callback, priv->next_buffer);
      return -EINVAL;
    }

  memset(&csi_config, 0, sizeof(csi_config));
  csi_config.ctlr_id = CAMERA_CSI_ID;
  csi_config.clk_src = MIPI_CSI_PHY_CLK_SRC_DEFAULT;
  csi_config.h_res = CAMERA_WIDTH;
  csi_config.v_res = CAMERA_HEIGHT;
  csi_config.data_lane_num = CAMERA_LANE_NUM;
  csi_config.lane_bit_rate_mbps = CAMERA_LANE_BITRATE;
  /* ISP converts SC2336 RAW10 to RGB565.  CSI DMA then sees the ISP
   * output, so both CSI colors must be RGB565.  RAW10->RGB565 is not a
   * supported CSI-bridge conversion on this HAL and fails CSI create,
   * leaking the 1.8MB backup buffer.
   */
  csi_config.input_data_color_type = CAM_CTLR_COLOR_RGB565;
  csi_config.output_data_color_type = CAM_CTLR_COLOR_RGB565;
  csi_config.queue_items = 2;

  ret = esp_cam_new_csi_ctlr(&csi_config, &priv->csi);
  if (ret != ESP_OK)
    {
      printf("Camera: esp_cam_new_csi_ctlr failed: %d\n", ret);
      priv->csi = NULL;
      return -EIO;
    }

  memset(&callbacks, 0, sizeof(callbacks));
  callbacks.on_get_new_trans = camera_on_new_trans;
  callbacks.on_trans_finished = camera_on_finished;
  ret = esp_cam_ctlr_register_event_callbacks(priv->csi, &callbacks, priv);
  printf("Camera: csi_new ok cbs=%d buf=%p\n", ret, priv->next_buffer);

  memset(&isp_config, 0, sizeof(isp_config));
  isp_config.clk_src = ISP_CLK_SRC_DEFAULT;
  isp_config.clk_hz = 80 * 1000 * 1000;
  isp_config.input_data_source = ISP_INPUT_DATA_SOURCE_CSI;
  isp_config.input_data_color_type = ISP_COLOR_RAW10;
  isp_config.output_data_color_type = ISP_COLOR_RGB565;
  isp_config.yuv_range = ISP_COLOR_RANGE_FULL;
  isp_config.yuv_std = ISP_YUV_CONV_STD_BT601;
  isp_config.h_res = CAMERA_WIDTH;
  isp_config.v_res = CAMERA_HEIGHT;
  isp_config.bayer_order = COLOR_RAW_ELEMENT_ORDER_BGGR;

  if (ret == ESP_OK)
    {
      ret = esp_cam_ctlr_enable(priv->csi);
      printf("Camera: csi_enable=%d\n", ret);
    }
  if (ret == ESP_OK)
    {
      ret = esp_isp_new_processor(&isp_config, &priv->isp);
      printf("Camera: isp_new=%d\n", ret);
    }
  if (ret == ESP_OK)
    {
      ret = camera_isp_pipeline_enable(priv);
      printf("Camera: isp_pipeline=%d\n", ret);
    }
  if (ret == ESP_OK)
    {
      ret = esp_isp_enable(priv->isp);
      printf("Camera: isp_enable=%d\n", ret);
    }
  if (ret == ESP_OK)
    {
      ret = esp_cam_ctlr_start(priv->csi);
      printf("Camera: csi_start=%d buf=%p\n", ret, priv->next_buffer);
    }
  if (ret != ESP_OK)
    {
      printf("Camera: CSI/ISP start failed: %d\n", ret);
      if (priv->isp != NULL)
        {
          camera_isp_pipeline_disable(priv);
          (void)esp_isp_disable(priv->isp);
          (void)esp_isp_del_processor(priv->isp);
          priv->isp = NULL;
        }

      if (priv->csi != NULL)
        {
          (void)esp_cam_ctlr_stop(priv->csi);
          (void)esp_cam_ctlr_disable(priv->csi);
          (void)esp_cam_ctlr_del(priv->csi);
        }

      priv->csi = NULL;
      return -EIO;
    }

  priv->callback = callback;
  priv->callback_arg = arg;
  priv->streaming = true;
  printf("Camera: CSI/ISP started RGB565 %dx%d\n",
         CAMERA_WIDTH, CAMERA_HEIGHT);
  return 0;
}

static int camera_data_stop(FAR struct imgdata_s *data)
{
  FAR struct esp32p4_camera_s *priv =
    container_of(data, struct esp32p4_camera_s, data);

  priv->streaming = false;
  priv->next_buffer = NULL;
  priv->callback = NULL;
  priv->callback_arg = NULL;

  if (priv->csi != NULL)
    {
      (void)esp_cam_ctlr_stop(priv->csi);
      (void)esp_cam_ctlr_disable(priv->csi);
      (void)esp_cam_ctlr_del(priv->csi);
      priv->csi = NULL;
    }

  if (priv->isp != NULL)
    {
      camera_isp_pipeline_disable(priv);
      (void)esp_isp_disable(priv->isp);
      (void)esp_isp_del_processor(priv->isp);
      priv->isp = NULL;
    }

  return 0;
}

static void *camera_data_alloc(FAR struct imgdata_s *data,
                               uint32_t align_size, uint32_t size)
{
  (void)data;

  if (align_size < CAMERA_BUF_ALIGN)
    {
      align_size = CAMERA_BUF_ALIGN;
    }

  return kumm_memalign(align_size, size);
}

static void camera_data_free(FAR struct imgdata_s *data, FAR void *addr)
{
  kumm_free(addr);
}

static const struct imgdata_ops_s g_data_ops =
{
  .init                   = camera_data_init,
  .uninit                 = camera_data_uninit,
  .set_buf                = camera_data_set_buf,
  .validate_frame_setting = camera_data_validate,
  .start_capture          = camera_data_start,
  .stop_capture           = camera_data_stop,
  .alloc                  = camera_data_alloc,
  .free                   = camera_data_free,
};

static const struct v4l2_fmtdesc g_fmtdescs[] =
{
  {
    .index = 0,
    .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .pixelformat = V4L2_PIX_FMT_RGB565,
    .description = "RGB565",
  }
};

static const struct v4l2_frmsizeenum g_frmsizes[] =
{
  {
    .index = 0,
    .pixel_format = V4L2_PIX_FMT_RGB565,
    .type = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete = { .width = CAMERA_WIDTH, .height = CAMERA_HEIGHT },
  }
};

static const struct v4l2_frmivalenum g_frmintervals[] =
{
  {
    .index = 0,
    .pixel_format = V4L2_PIX_FMT_RGB565,
    .width = CAMERA_WIDTH,
    .height = CAMERA_HEIGHT,
    .type = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete = { .numerator = 1, .denominator = 30 },
  }
};

int board_camera_initialize(void)
{
  FAR struct imgsensor_s *sensor = &g_camera.sensor;
  uint8_t pid_h;
  uint8_t pid_l;
  int ret;

  memset(&g_camera, 0, sizeof(g_camera));
  g_camera.i2c = esp_i2cbus_initialize(CAMERA_I2C_PORT);
  if (g_camera.i2c == NULL)
    {
      syslog(LOG_ERR, "Camera: failed to initialize I2C%d\n",
             CAMERA_I2C_PORT);
      return -ENODEV;
    }

  if (camera_i2c_read(&g_camera, SC2336_REG_PID_H, &pid_h) < 0 ||
      camera_i2c_read(&g_camera, SC2336_REG_PID_L, &pid_l) < 0 ||
      (((uint16_t)pid_h << 8) | pid_l) != SC2336_PID)
    {
      syslog(LOG_ERR, "Camera: SC2336 not detected (PID %02x%02x)\n",
             pid_h, pid_l);
      return -ENODEV;
    }

  g_camera.ldo.chan_id = CAMERA_LDO_CHANNEL;
  g_camera.ldo.voltage_mv = CAMERA_LDO_MV;
  g_camera.sensor.ops = &g_sensor_ops;
  g_camera.sensor.fmtdescs_num = 1;
  g_camera.sensor.fmtdescs = g_fmtdescs;
  g_camera.sensor.frmsizes_num = 1;
  g_camera.sensor.frmsizes = g_frmsizes;
  g_camera.sensor.frmintervals_num = 1;
  g_camera.sensor.frmintervals = g_frmintervals;
  g_camera.data.ops = &g_data_ops;

  ret = capture_register("/dev/video0", &g_camera.data, &sensor, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Camera: capture_register failed: %d\n", ret);
    }

  return ret;
}

#endif /* CONFIG_ESP32P4_FUNCTION_EV_BOARD_CAMERA */

/**
 * Copyright (c) 2026, BlackBerry Limited. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "rpi_cam.hpp"

#include <spdlog/spdlog.h>
#include <unistd.h>

RpiCam::RpiCam(camera_unit_t camera_id) : id_(camera_id) {}

int RpiCam::init() {
  int err = camera_open(id_, CAMERA_MODE_RO, &handle_);
  if (err != EOK) {
    return err;
  }

  if (camera_has_feature(handle_, CAMERA_FEATURE_VFWINDOW) == false) {
    fprintf(stderr, "Viewfinder is not supported for this camera\n");
    return ENOTSUP;
  }

  camera_frametype_t frame_types[10];
  uint32_t frame_types_num = 0;
  camera_get_supported_vf_frame_types(handle_, 10, &frame_types_num,
                                      frame_types);
  int i = 0;
  for (; i < frame_types_num; i++) {
    if (frame_types[i] == CAMERA_FRAMETYPE_NV12 ||
        frame_types[i] == CAMERA_FRAMETYPE_YCBYCR) {
      spdlog::debug("Found supported frametype NV12 or YCBYCR");
      break;
    }
  }
  if (i == frame_types_num) {
    spdlog::error("Camera doesn't have supported frametype NV12 or YCBYCR");
    return ENOTSUP;
  }

  camera_set_vf_property(handle_, CAMERA_IMGPROP_FORMAT, frame_types[i]);
  camera_set_vf_property(handle_, CAMERA_IMGPROP_CREATEWINDOW, 0);
  camera_get_vf_property(handle_, CAMERA_IMGPROP_WIDTH, &width_,
                         CAMERA_IMGPROP_HEIGHT, &height_);
  return EOK;
}

int RpiCam::start(FrameCb callback) {
  if (handle_ == CAMERA_HANDLE_INVALID) {
    return ENODEV;
  }
  cb_ = callback;

  return camera_start_viewfinder(handle_, RpiCam::sf_cam_cb, NULL, this);
}

int RpiCam::stop() {
  camera_stop_viewfinder(handle_);
  camera_close(handle_);
  handle_ = CAMERA_HANDLE_INVALID;
  return EOK;
}

std::pair<int, int> RpiCam::get_screen_size() const {
  return std::make_tuple(width_, height_);
}

void RpiCam::sf_cam_cb(camera_handle_t handle, camera_buffer_t *buf,
                       void *data) {
  spdlog::trace("sf_cam_cb: callback");
  RpiCam *cam = reinterpret_cast<RpiCam *>(data);

  // Create our output image based on our camera info
  cv::Mat brg_img(cam->height_, cam->width_, CV_8UC3);
  if (buf->frametype == CAMERA_FRAMETYPE_NV12) {
    /**
     * @brief NV12 uses a 4:2:0 YUV format with 2 planes.
     *
     * The first plane is the Y plane (or grayscale) which is a 1 to 1 mapping
     * between the source and dst image this means we can just do a blind copy
     * of the entire image The second plan is the UV plane which contains 1 UV
     * entry for every 2x2 kernel in the src image. This means that there is
     * half has many lines and columns as the first plan (but the columns ends
     * up being equal as its a tuple of 2 values interlaced)
     *
     * The Kernel can be described with the following 4 pixels
     * Y    Y
     *   UV
     * Y    Y
     *
     * @note when creating the plans we provide the memory from camapi to avoid
     * the double copy
     */
    cv::Mat Y_img(buf->framedesc.nv12.height, buf->framedesc.nv12.width,
                  CV_8UC1, buf->framebuf);
    cv::Mat UV_img(buf->framedesc.nv12.height / 2,
                   buf->framedesc.nv12.width / 2, CV_8UC2,
                   buf->framebuf + buf->framedesc.nv12.uv_offset);
    cv::cvtColorTwoPlane(Y_img, UV_img, brg_img, cv::COLOR_YUV2BGR_NV12);
  } else if (buf->frametype == CAMERA_FRAMETYPE_YCBYCR) {
    cv::Mat yuyv_img(buf->framedesc.ycbycr.height, buf->framedesc.ycbycr.width,
                     CV_8UC2, buf->framebuf);
    cv::cvtColor(yuyv_img, brg_img, cv::COLOR_YUV2BGR_YUY2);
  } else {
    spdlog::warn("Unsupported Frametype {}", static_cast<int>(buf->frametype));
    return;
  }

  if (cam->cb_) {
    cam->cb_(brg_img);
  } else {
    spdlog::warn("No callback set for RpiCam");
  }
}

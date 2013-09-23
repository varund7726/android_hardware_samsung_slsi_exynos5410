/*
 * Copyright (C) 2013, The CyanogenMod Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <sys/types.h>

#include <utils/Log.h>

#include "exynos_format.h"
#include "ExynosCameraHWInterface.h"

//#define LOG_NDEBUG 0
#define LOG_TAG "ExynosCameraHWInterface"

namespace android {

/* Helper functions */

int hw_int_convert_format(int v4l2_format)
{
    switch (v4l2_format) {
        case V4L2_PIX_FMT_NV21:
            return HAL_PIXEL_FORMAT_YCrCb_420_SP;
        case V4L2_PIX_FMT_YUV420:
            return HAL_PIXEL_FORMAT_YV12;
        case V4L2_PIX_FMT_RGB565:
            return HAL_PIXEL_FORMAT_RGB_565;
        case V4L2_PIX_FMT_RGB32:
            return HAL_PIXEL_FORMAT_RGBX_8888;
        default:
            ALOGW("WARN(%s): unknown V4L2 format %d, returning YCrCb420SP",
                    __FUNCTION__, v4l2_format);
            return HAL_PIXEL_FORMAT_YCrCb_420_SP;
    }
}

void hw_int_set_buffer_size(int format, int w, int h, ExynosBuffer *buf)
{
    switch (format) {
        // 1 plane
        case V4L2_PIX_FMT_RGB565:
        case V4L2_PIX_FMT_YUYV:
        case V4L2_PIX_FMT_UYVY:
        case V4L2_PIX_FMT_VYUY:
        case V4L2_PIX_FMT_YVYU:
            buf->size.extS[0] = FRAME_SIZE(
                    V4L2_PIX_2_HAL_PIXEL_FORMAT(format), w, h);
            buf->size.extS[1] = 0;
            buf->size.extS[2] = 0;
            break;
        // 2 plane
        case V4L2_PIX_FMT_NV12:
        case V4L2_PIX_FMT_NV12T:
        case V4L2_PIX_FMT_NV21:
            buf->size.extS[0] = ALIGN(w, 16) * ALIGN(h, 16);
            buf->size.extS[1] = ALIGN(w / 2, 16) * ALIGN(h / 2, 16);
            buf->size.extS[2] = 0;
            break;
        case V4L2_PIX_FMT_NV12M:
        case V4L2_PIX_FMT_NV12MT_16X16:
            buf->size.extS[0] = ALIGN(ALIGN(w, 16) * ALIGN(h, 16), 2048);
            buf->size.extS[1] = ALIGN(ALIGN(w, 16) * ALIGN(h >> 1, 8), 2048);
            buf->size.extS[2] = 0;
            break;
        case V4L2_PIX_FMT_NV16:
        case V4L2_PIX_FMT_NV61:
            buf->size.extS[0] = ALIGN(w, 16) * ALIGN(h, 16);
            buf->size.extS[1] = ALIGN(w, 16) * ALIGN(h, 16);
            buf->size.extS[2] = 0;
            break;
        // 3 plane
        case V4L2_PIX_FMT_YUV420:
        case V4L2_PIX_FMT_YVU420:
            buf->size.extS[0] = (w * h);
            buf->size.extS[1] = (w * h) >> 2;
            buf->size.extS[2] = (w * h) >> 2;
            break;
        case V4L2_PIX_FMT_YUV420M:
        case V4L2_PIX_FMT_YVU420M:
        case V4L2_PIX_FMT_YUV422P:
            buf->size.extS[0] = ALIGN(w, 16) * ALIGN(h, 16);
            buf->size.extS[1] = ALIGN(w / 2, 8) * ALIGN(h / 2, 8);
            buf->size.extS[2] = ALIGN(w / 2, 8) * ALIGN(h / 2, 8);
            break;
        default:
            ALOGE("ERR(%s): unmatched color format (%d)", __FUNCTION__, format);
            break;
    }
}

/* Internal functions */

gralloc_module_t const *ExynosCameraHWInterface::m_grallocHal;

bool ExynosCameraHWInterface::m_previewThreadProcessBuffers(void)
{
    ExynosBuffer buf;
    buffer_handle_t *bufHandle;
    buffer_params_t *previewBuf;
    int width = m_camera->getPreviewWidth();
    int height = m_camera->getPreviewHeight();
    bool foundBuf;
    int ret;
    int idx;
    int stride;
    void *vaddr[3];

    if (!m_previewWindow) {
        ALOGE("ERR(%s) preview window not initialized!", __FUNCTION__);
        return false;
    }

    if (!m_grallocHal) {
        ALOGE("ERR(%s): gralloc not initialized!", __FUNCTION__);
        return false;
    }

    /* Get buffer */
    ret = m_camera->getPreviewBuffer(&buf);
    if (ret != 0) {
        ALOGE("ERR(%s): getPreviewBuffer failed, err = %d", __FUNCTION__, ret);
        return false;
    }

    idx = buf.reserved.p;
    previewBuf = &m_previewBuffers[idx];

    /* Get gralloc buffer */
    if (previewBuf->locked) {
        m_grallocHal->unlock(m_grallocHal, *previewBuf->handle);
        previewBuf->locked = false;
    } else {
        ret = m_previewWindow->lock_buffer(m_previewWindow, bufHandle);
        if (ret != 0) {
            ALOGE("ERR(%s): failed to lock gralloc buffer, err = %d",
                    __FUNCTION__, ret);
        }
    }

    /* Enqueue buffer */
    if (previewBuf->available) {
        ret = m_previewWindow->enqueue_buffer(m_previewWindow,
                previewBuf->handle);
        if (ret != 0) {
            ALOGE("ERR(%s): failed to enqueue buffer[%d], err = %d",
                    __FUNCTION__, idx, ret);
            return false;
        }
        previewBuf->available = false;
    }

    /* Dequeue gralloc buffer into bufHandle */
    ret = m_previewWindow->dequeue_buffer(m_previewWindow, &bufHandle, &stride);
    if (ret != 0) {
        ALOGE("ERR(%s): failed to dequeue buffer, err = %d",
                __FUNCTION__, ret);
        return false;
    }

    /* Get dequeued buffer virtual address */
    ret = m_grallocHal->lock(m_grallocHal, *bufHandle,
            GRALLOC_USAGE_SW_WRITE_OFTEN, 0, 0, width, height, vaddr);
    if (ret != 0) {
        ALOGE("ERR(%s): failed to lock gralloc buffer, err = %d",
                __FUNCTION__, ret);
        return false;
    }

    /* Update m_previewBuffers */
    const private_handle_t *priv_handle =
        reinterpret_cast<const private_handle_t *>(*bufHandle);

    foundBuf = false;
    for (int i = 0; i < m_camera->getPreviewNumBuffers(); i++) {
        if ((unsigned int)m_previewBuffers[i].vaddr == (unsigned int)vaddr[0]) {
            foundBuf = true;

            m_previewBuffers[i].handle = bufHandle;
            m_previewBuffers[i].stride = stride;

            buf.reserved.p = i;
            buf.virt.extP[0] = (char *) vaddr[0];
            buf.virt.extP[1] = (char *) vaddr[1];
            buf.virt.extP[2] = (char *) vaddr[2];

            buf.fd.extFd[0] = priv_handle->fd;
            buf.fd.extFd[1] = priv_handle->fd1;
            buf.fd.extFd[2] = priv_handle->fd2;

            m_camera->setPreviewBuffer(&buf);
            m_previewBuffers[i].available = true;
            break;
        }
    }

    if (!foundBuf) {
        ALOGE("ERR(%s): failed to find buffer %p", __FUNCTION__, vaddr[0]);
        return false;
    }

    ret = m_camera->putPreviewBuffer(&buf);
    if (ret != 0) {
        ALOGE("ERR(%s): putPreviewBuffer failed, err = %d",
                __FUNCTION__, ret);
        return false;
    }

    return true;
}

bool ExynosCameraHWInterface::m_previewThreadProcessCallbacks()
{
    return false;
}

bool ExynosCameraHWInterface::m_previewThreadFunc()
{
    ALOGV("DEBUG(%s): starting", __FUNCTION__);

    while (true) {
        if (m_previewExit) {
            // TODO: stop preview
            return true;
        }

        if (m_previewRunning) {
            m_previewThreadProcessBuffers();
            m_previewThreadProcessCallbacks();
        }
    }
}

/* Public functions */

ExynosCameraHWInterface::ExynosCameraHWInterface(int cameraId,
        camera_device_t *dev) :
    m_cameraId(cameraId),
    m_minUndequeuedBufs(0),
    m_previewRunning(false),
    m_previewExit(false)
{
    int ret;

    ALOGV("DEBUG(%s):" __FUNCTION__);

    if (!m_grallocHal) {
        ret = hw_get_module(GRALLOC_HARDWARE_MODULE_ID,
                (const hw_module_t **) &m_grallocHal);
        if (ret != 0) {
            ALOGE("ERR(%s): Failed to load gralloc HAL", __FUNCTION__);
        }
    }
}

ExynosCameraHWInterface::~ExynosCameraHWInterface()
{

}

status_t ExynosCameraHWInterface::setPreviewWindow(preview_stream_ops *w)
{
    int ret;
    int width, height, fmt, num_buf;

    ALOGV("DEBUG(%s): preview window %p", __FUNCTION__, w);

    if (!w) {
        m_previewWindow = NULL;
        ALOGE("ERR(%s): preview window is NULL!", __FUNCTION__);
        return OK;
    }

    ret = w->get_min_undequeued_buffer_count(w, &m_minUndequeuedBufs);
    if (ret != 0) {
        ALOGE("ERR(%s): could not retrieve minimum undequeued buffer count",
                __FUNCTION__);
        return -EINVAL;
    }

    num_buf = m_camera->getPreviewNumBuffers();
    if (num_buf <= m_minUndequeuedBufs) {
        ALOGE("ERR(%s): minimum undequeued buffer count %d too high!",
                __FUNCTION__, m_minUndequeuedBufs);
        return -EINVAL;
    }

    ret = w->set_buffer_count(w, num_buf);
    if (ret != 0) {
        ALOGE("ERR(%s): failed to set buffer count to %d",
                __FUNCTION__, num_buf);
        return -EINVAL;
    }

    ret = w->set_usage(w, GRALLOC_USAGE_SW_WRITE_OFTEN);
    if (ret != 0) {
        ALOGE("ERR(%s): failed to set gralloc usage", __FUNCTION__);
        return -EINVAL;
    }

    width = m_camera->getPreviewWidth();
    height = m_camera->getPreviewHeight();
    fmt = hw_int_convert_format(m_camera->getPreviewColorFormat());
    ret = w->set_buffers_geometry(w, width, height, fmt);
    if (ret != 0) {
        ALOGE("ERR(%s): failed to set buffer geometry: %dx%d, fmt: %d",
                __FUNCTION__, width, height, fmt);
        return -EINVAL;
    }

    m_previewWindow = w;

    return 0;
}

void ExynosCameraHWInterface::setCallbacks(camera_notify_callback notify_cb,
        camera_data_callback data_cb,
        camera_data_timestamp_callback data_cb_timestamp,
        camera_request_memory get_memory,
        void *user)
{
    m_notifyCb = notify_cb;
    m_dataCb = data_cb;
    m_dataCbTimestamp = data_cb_timestamp;
    m_getMemoryCb = get_memory;
    m_callbackCookie = user;
}

void ExynosCameraHWInterface::enableMsgType(int32_t msgType)
{
    m_msgEnabled |= msgType;
}

void ExynosCameraHWInterface::disableMsgType(int32_t msgType)
{
    m_msgEnabled &= (~msgType);
}

bool ExynosCameraHWInterface::msgTypeEnabled(int32_t msgType)
{
    return ((m_msgEnabled & msgType) > 0);
}

status_t ExynosCameraHWInterface::startPreview()
{
    ExynosBuffer buf;
    void *vaddr[3];
    int fd[3];
    int numBuffers;
    int format, width, height;
    int ret;
    buffer_params_t *previewBuf;

    for (int i = 0; i < 3; i++) {
        vaddr[i] = NULL;
        fd[i] = 0;
    }

    format = m_camera->getPreviewColorFormat();
    width = m_camera->getPreviewWidth();
    height = m_camera->getPreviewHeight();
    numBuffers = m_camera->getPreviewNumBuffers();
    for (int i = 0; i < numBuffers; i++) {
        previewBuf = &m_previewBuffers[i];

        previewBuf->available = false;

        ret = m_previewWindow->dequeue_buffer(m_previewWindow,
                &previewBuf->handle, &previewBuf->stride);
        if (ret != 0) {
            ALOGE("ERR(%s): could not dequeue gralloc buffer[%d]!!",
                    __FUNCTION__, i);
            continue;
        }

        ret = m_previewWindow->lock_buffer(m_previewWindow, previewBuf->handle);
        if (ret != 0) {
            ALOGE("ERR(%s): could not lock gralloc buffer[%d]!!",
                    __FUNCTION__, i);
        }

        if (!previewBuf->locked) {
            ret = m_grallocHal->lock(m_grallocHal, *previewBuf->handle,
                    GRALLOC_USAGE_SW_WRITE_OFTEN, 0, 0, width, height, vaddr);
            if (ret != 0) {
                ALOGE("ERR(%s): Could not obtain gralloc buffer", __FUNCTION__);
                ret = m_previewWindow->cancel_buffer(m_previewWindow,
                        previewBuf->handle);
                if (ret != 0) {
                    ALOGE("ERR(%s): Could not cancel buffer[%d]!!",
                            __FUNCTION__, i);
                }
                continue;
            }

            const private_handle_t *priv_handle =
                reinterpret_cast<const private_handle_t *>(previewBuf->handle);
            fd[0] = priv_handle->fd;
            fd[1] = priv_handle->fd1;
            fd[2] = priv_handle->fd2;
            previewBuf->vaddr = vaddr[0];
            previewBuf->locked = true;
        }

        // get buf->buffer size from format, width, height
        hw_int_set_buffer_size(format, width, height, &buf);

        buf.reserved.p = i;
        buf.virt.extP[0] = (char *) vaddr[0];
        buf.virt.extP[1] = (char *) vaddr[1];
        buf.virt.extP[2] = (char *) vaddr[2];
        buf.fd.extFd[0] = fd[0];
        buf.fd.extFd[1] = fd[1];
        buf.fd.extFd[2] = fd[2];

        m_camera->setPreviewBuffer(&buf);

        // TODO: allocate preview heaps??

        previewBuf->available = true;
    }

    for (int i = numBuffers - m_minUndequeuedBufs; i < numBuffers; i++) {
        m_camera->getPreviewBuffer(&buf);

        previewBuf = &m_previewBuffers[buf.reserved.p];
        if (m_grallocHal && previewBuf->locked) {
            m_grallocHal->unlock(m_grallocHal, *previewBuf->handle);
            previewBuf->locked = false;
        }

        ret = m_previewWindow->cancel_buffer(m_previewWindow,
                previewBuf->handle);
        if (ret != 0) {
            ALOGE("ERR(%s): Could not cancel buffer[%d]!!",
                    __FUNCTION__, buf.reserved.p);
        }

        previewBuf->available = false;
    }

    m_previewRunning = true;

    return OK;
}

void ExynosCameraHWInterface::stopPreview()
{

}

int ExynosCameraHWInterface::getCameraId()
{
    return m_cameraId;
}

}; // namespace android

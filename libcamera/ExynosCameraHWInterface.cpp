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

#include "ExynosCameraHWInterface.h"

//#define LOG_NDEBUG 0
#define LOG_TAG "ExynosCameraHWInterface"

namespace android {

ExynosCameraHWInterface::ExynosCameraHWInterface(int cameraId, camera_device_t *dev)
{

}

ExynosCameraHWInterface::~ExynosCameraHWInterface()
{

}

status_t ExynosCameraHWInterface::setPreviewWindow(preview_stream_ops *ops)
{
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

status_t ExynosCameraHWInterface::startPreview()
{
    return 0;
}

status_t ExynosCameraHWInterface::stopPreview()
{
    return 0;
}

}; // namespace android

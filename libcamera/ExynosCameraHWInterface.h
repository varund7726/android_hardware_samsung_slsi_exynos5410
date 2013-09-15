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

#ifdef _EXYNOS_CAMERA_HW_INTERFACE_H_
#define _EXYNOS_CAMERA_HW_INTERFACE_H_

#include <hardware/camera.h>

namespace android {

enum sensor_name {
    SENSOR_NAME_S5K3H2  = 1,
    SENSOR_NAME_S5K6A3  = 2,
    SENSOR_NAME_S5K4E5  = 3,
    SENSOR_NAME_S5K3H7  = 4,
    SENSOR_NAME_CUSTOM  = 5,
    SENSOR_NAME_UNKWN0  = 6,
    SENSOR_NAME_UNKWN1  = 7,
    SENSOR_NAME_IMX135  = 8,
    SENSOR_NAME_S5K6B2  = 9,
    SENSOR_NAME_END
};

class ExynosCameraHWInterface : public virtual RefBase {
public:
    ExynosCameraHWInterface(int cameraId, camera_device_t *dev);
    virtual                 ~ExynosCameraHWInterface();

    virtual void            setCallbacks(camera_notify_callback notify_cb,
                                    camera_data_callback data_cb,
                                    camera_data_timestamp_callback data_cb_timestamp,
                                    camera_request_memory get_memory,
                                    void *user);


private:
    camera_notify_callback  m_notifyCb;
    camera_data_callback    m_dataCb;
    camera_data_timestamp_callback m_dataCbTimestamp;
    camera_request_memory   m_getMemoryCb;
    void                    *m_callbackCookie;

};

}; // namespace android

#endif /* _EXYNOS_CAMERA_HW_INTERFACE_H_ */

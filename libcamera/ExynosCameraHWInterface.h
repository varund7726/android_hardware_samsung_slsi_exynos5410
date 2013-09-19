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

#ifndef _EXYNOS_CAMERA_HW_INTERFACE_H_
#define _EXYNOS_CAMERA_HW_INTERFACE_H_

#include <utils/threads.h>
#include <utils/RefBase.h>

#include <hardware/camera.h>
#include <hardware/gralloc.h>

#include "gralloc_priv.h"

#include "ExynosCamera.h"

namespace android {

/* tracks gralloc buffer parameters */
typedef struct buffer_params {
    void                   *vaddr;
    int                     stride;
    bool                    locked;
    bool                    available;
    buffer_handle_t        *handle;
} buffer_params_t;

class ExynosCameraHWInterface : public virtual RefBase {
public:
    ExynosCameraHWInterface(int cameraId, camera_device_t *dev);
    virtual                 ~ExynosCameraHWInterface();

    virtual status_t        setPreviewWindow(preview_stream_ops *ops);
    virtual void            setCallbacks(camera_notify_callback notify_cb,
                                    camera_data_callback data_cb,
                                    camera_data_timestamp_callback data_cb_timestamp,
                                    camera_request_memory get_memory,
                                    void *user);
    virtual status_t        startPreview();
    virtual void            stopPreview();

    int                     getCameraId();


private:
    /* Internal classes */
    class PreviewThread : public Thread {
    public:
        PreviewThread(ExynosCameraHWInterface *hw) :
            Thread(false),
            m_hardware(hw)
        {
        }

        virtual void onFirstRef()
        {
            run("PreviewThread", PRIORITY_DEFAULT);
        }

        virtual bool threadLoop()
        {
            m_hardware->m_previewThreadFunc();
            return false;
        }
    private:
        ExynosCameraHWInterface *m_hardware;
    };

    /* Internal functions */
    bool                    m_previewThreadFunc(void);
    bool                    m_previewThreadProcessBuffers(void);
    bool                    m_previewThreadProcessCallbacks(void);

    /* Internal variables */
    int                     m_cameraId;
    ExynosCamera           *m_camera;

    preview_stream_ops     *m_previewWindow;
    int                     m_minUndequeuedBufs;

    buffer_params_t         m_previewBuffers[NUM_MAX_CAMERA_BUFFERS];

    bool                    m_previewRunning;
    bool                    m_previewExit;

    camera_notify_callback  m_notifyCb;
    camera_data_callback    m_dataCb;
    camera_data_timestamp_callback m_dataCbTimestamp;
    camera_request_memory   m_getMemoryCb;
    void                    *m_callbackCookie;

    static gralloc_module_t const *m_grallocHal;
};

}; // namespace android

#endif /* _EXYNOS_CAMERA_HW_INTERFACE_H_ */

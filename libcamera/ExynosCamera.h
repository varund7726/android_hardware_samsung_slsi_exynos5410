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

#ifndef _EXYNOS_CAMERA_H_
#define _EXYNOS_CAMERA_H_

#include <hardware/camera.h>
#include <utils/RefBase.h>

#include "exynos_v4l2.h"
#include "ExynosBuffer.h"
#include "ion.h"
#include "videodev2_exynos_camera.h"

#include "ExynosCameraInfo.h"

namespace android {

#define NODE_PREFIX                 "/dev/video"
#define VIDEO_NODE_SENSOR0          (40)
#define VIDEO_NODE_SENSOR1          (41)
#define VIDEO_NODE_IS3A0            (42)
#define VIDEO_NODE_IS3A1            (43)
#define VIDEO_NODE_ISP              (44)
#define VIDEO_NODE_SCALERC          (45)
#define VIDEO_NODE_SCALERP          (46)

#define NUM_MAX_CAMERA_BUFFERS      (16)
#define NUM_SENSOR_BUFFERS          (8)
#define NUM_SCC_BUFFERS             (8)
#define NUM_SCP_BUFFERS             (8)
#define NUM_MIN_SENSOR_QBUF         (3)

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

typedef struct node_buffer {
    bool valid;
    ExynosBuffer buf;
} node_buffer_t;

typedef struct node_info {
    int fd;
    int width;
    int height;
    int format;
    int planes;
    int buffers;
    enum v4l2_memory memory;
    enum v4l2_buf_type type;
    node_buffer_t buffer[NUM_MAX_CAMERA_BUFFERS];
    int status;
} node_info_t;

typedef struct camera_hw_info {
    int sensor_id;

    node_info_t sensor0;
    node_info_t sensor1;
    node_info_t isp;
    node_info_t is3a0;
    node_info_t is3a1;
    node_info_t scc; // capture
    node_info_t scp; // preview
} camera_hw_info_t;

class ExynosCamera : public virtual RefBase {

private:
    ExynosCamera();
    virtual ~ExynosCamera();

public:
    static ExynosCamera *createInstance(void)
    {
        static ExynosCamera singleton;
        return &singleton;
    }

    bool                    create(int cameraId);
    bool                    destroy(void);
    int                     getCameraId(void);

    int                     getPreviewWidth(void);
    int                     getPreviewHeight(void);
    int                     getPreviewColorFormat(void);
    int                     getHardwarePreviewWidth(void);
    int                     getHardwarePreviewHeight(void);
    int                     getHardwarePreviewColorFormat(void);
    int                     getPreviewNumBuffers(void);

    bool                    setPreviewBuffer(ExynosBuffer *buf);
    bool                    getPreviewBuffer(ExynosBuffer *buf);
    bool                    putPreviewBuffer(ExynosBuffer *buf);

private:
    bool                    initializeIspChain(void);

    int                     m_cameraId;
    ExynosCameraInfo       *m_cameraInfo;
    camera_hw_info_t        m_streamInfo;

    int                     m_previewWidth;
    int                     m_previewHeight;
    int                     m_previewColorFormat;

    ion_client              m_ionClient;
};

}; // namespace android

#endif /* _EXYNOS_CAMERA_H_ */

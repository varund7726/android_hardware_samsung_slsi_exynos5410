/*
**
** Copyright 2008, The Android Open Source Project
** Copyright 2012, Samsung Electronics Co. LTD
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/

/*!
 * \file      ExynosCamera2.h
 * \brief     header file for static information of camera2
 * \author    Sungjoong Kang(sj3.kang@samsung.com)
 * \date      2012/08/06
 *
 * <b>Revision History: </b>
 * - 2012/08/06 : Sungjoong Kang(sj3.kang@samsung.com) \n
 *   Initial Release
 *
 */

#include <hardware/camera2.h>
#include <camera/Camera.h>
#include <camera/CameraParameters.h>
#include "exynos_format.h"
#include "fimc-is-metadata.h"

namespace android {

struct ExynosCamera2Info
{
public:
    uint32_t        sensorW;
    uint32_t        sensorH;
    uint32_t        sensorRawW;
    uint32_t        sensorRawH;
    int             numScalerResolution;
    const uint32_t  *scalerResolutions;
    int             numJpegResolution;
    const uint32_t  *jpegResolutions;
    int             numThumbnailResolution;
    const uint32_t  *thumbnailResolutions;
    float           minFocusDistance;
    float           focalLength;
    float           aperture;
    float           fnumber;
    const uint8_t   *availableAfModes;
    const uint8_t   *sceneModeOverrides;
    const uint8_t   *availableAeModes;
    int             numAvailableAfModes;
    int             numSceneModeOverrides;
    int             numAvailableAeModes;

    /* FD information */
    int32_t    maxFaceCount;
};

struct ExynosCamera2InfoS5K4E5 : public ExynosCamera2Info
{
public:
    ExynosCamera2InfoS5K4E5();
	~ExynosCamera2InfoS5K4E5();
};

struct ExynosCamera2InfoS5K6A3 : public ExynosCamera2Info
{
public:
    ExynosCamera2InfoS5K6A3();
	~ExynosCamera2InfoS5K6A3();
};

class ExynosCamera2 {
public:
    ExynosCamera2(int cameraId);
    ~ExynosCamera2();

    ExynosCamera2Info  *m_curCameraInfo;

    uint32_t getSensorW();
    uint32_t getSensorH();
    uint32_t getSensorRawW();
    uint32_t getSensorRawH();

    bool isSupportedResolution(uint32_t width, uint32_t height);
    bool isSupportedJpegResolution(uint32_t width, uint32_t height);
    bool isSupportedThumbnailResolution(uint32_t width, uint32_t height);

    status_t constructStaticInfo(camera_metadata_t **info,
        int cameraId, bool sizeRequest);

    status_t constructDefaultRequest(int request_template,
        camera_metadata_t **request, bool sizeRequest);
    int     m_cameraId;
};
}

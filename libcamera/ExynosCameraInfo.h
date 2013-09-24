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

#ifndef _EXYNOS_CAMERA_INFO_H_
#define _EXYNOS_CAMERA_INFO_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <sys/types.h>

#include <camera/CameraParameters.h>

#include <hardware/camera.h>

namespace android {

//! Camera ID
enum CAMERA_ID {
    CAMERA_ID_BACK  = 0,   //!<
    CAMERA_ID_FRONT = 1,   //!<
};

//! Anti banding
enum {
    ANTIBANDING_AUTO = (1 << 0), //!< \n
    ANTIBANDING_50HZ = (1 << 1), //!< \n
    ANTIBANDING_60HZ = (1 << 2), //!< \n
    ANTIBANDING_OFF  = (1 << 3), //!< \n
};

//! Effect
enum {
    EFFECT_NONE       = (1 << 0), //!< \n
    EFFECT_MONO       = (1 << 1), //!< \n
    EFFECT_NEGATIVE   = (1 << 2), //!< \n
    EFFECT_SOLARIZE   = (1 << 3), //!< \n
    EFFECT_SEPIA      = (1 << 4), //!< \n
    EFFECT_POSTERIZE  = (1 << 5), //!< \n
    EFFECT_WHITEBOARD = (1 << 6), //!< \n
    EFFECT_BLACKBOARD = (1 << 7), //!< \n
    EFFECT_AQUA       = (1 << 8), //!< \n
};

//! Flash mode
enum {
    FLASH_MODE_OFF     = (1 << 0), //!< \n
    FLASH_MODE_AUTO    = (1 << 1), //!< \n
    FLASH_MODE_ON      = (1 << 2), //!< \n
    FLASH_MODE_RED_EYE = (1 << 3), //!< \n
    FLASH_MODE_TORCH   = (1 << 4), //!< \n
};

//! Focus mode
enum {
    FOCUS_MODE_AUTO               = (1 << 0), //!< \n
    FOCUS_MODE_INFINITY           = (1 << 1), //!< \n
    FOCUS_MODE_MACRO              = (1 << 2), //!< \n
    FOCUS_MODE_FIXED              = (1 << 3), //!< \n
    FOCUS_MODE_EDOF               = (1 << 4), //!< \n
    FOCUS_MODE_CONTINUOUS_VIDEO   = (1 << 5), //!< \n
    FOCUS_MODE_CONTINUOUS_PICTURE = (1 << 6), //!< \n
    FOCUS_MODE_TOUCH              = (1 << 7), //!< \n
    FOCUS_MODE_CONTINUOUS_PICTURE_MACRO = (1 << 8), //!< \n
};

//! Focus distance
enum {
    FOCUS_DISTANCE_10           = (1 << 0), //!< \n
    FOCUS_DISTANCE_120          = (1 << 1), //!< \n
    FOCUS_DISTANCE_INF          = (1 << 2), //!< \n
};

//! Scene mode
enum {
    SCENE_MODE_AUTO           = (1 << 0), //!< \n
    SCENE_MODE_ACTION         = (1 << 1), //!< \n
    SCENE_MODE_PORTRAIT       = (1 << 2), //!< \n
    SCENE_MODE_LANDSCAPE      = (1 << 3), //!< \n
    SCENE_MODE_NIGHT          = (1 << 4), //!< \n
    SCENE_MODE_NIGHT_PORTRAIT = (1 << 5), //!< \n
    SCENE_MODE_THEATRE        = (1 << 6), //!< \n
    SCENE_MODE_BEACH          = (1 << 7), //!< \n
    SCENE_MODE_SNOW           = (1 << 8), //!< \n
    SCENE_MODE_SUNSET         = (1 << 9), //!< \n
    SCENE_MODE_STEADYPHOTO    = (1 << 10), //!< \n
    SCENE_MODE_FIREWORKS      = (1 << 11), //!< \n
    SCENE_MODE_SPORTS         = (1 << 12), //!< \n
    SCENE_MODE_PARTY          = (1 << 13), //!< \n
    SCENE_MODE_CANDLELIGHT    = (1 << 14), //!< \n
};

//! White balance
enum {
    WHITE_BALANCE_AUTO             = (1 << 0), //!< \n
    WHITE_BALANCE_INCANDESCENT     = (1 << 1), //!< \n
    WHITE_BALANCE_FLUORESCENT      = (1 << 2), //!< \n
    WHITE_BALANCE_WARM_FLUORESCENT = (1 << 3), //!< \n
    WHITE_BALANCE_DAYLIGHT         = (1 << 4), //!< \n
    WHITE_BALANCE_CLOUDY_DAYLIGHT  = (1 << 5), //!< \n
    WHITE_BALANCE_TWILIGHT         = (1 << 6), //!< \n
    WHITE_BALANCE_SHADE            = (1 << 7), //!< \n
};

//! Jpeg Quality
enum JPEG_QUALITY {
    JPEG_QUALITY_MIN        = 0,    //!<
    JPEG_QUALITY_ECONOMY    = 70,   //!<
    JPEG_QUALITY_NORMAL     = 80,   //!<
    JPEG_QUALITY_SUPERFINE  = 90,   //!<
    JPEG_QUALITY_MAX        = 100,  //!<
};

//! Metering
enum {
    METERING_MODE_AVERAGE = (1 << 0), //!< \n
    METERING_MODE_CENTER  = (1 << 1), //!< \n
    METERING_MODE_MATRIX  = (1 << 2), //!< \n
    METERING_MODE_SPOT    = (1 << 3), //!< \n
};

//! Contrast
enum {
    CONTRAST_AUTO    = (1 << 0), //!< \n
    CONTRAST_MINUS_2 = (1 << 1), //!< \n
    CONTRAST_MINUS_1 = (1 << 2), //!< \n
    CONTRAST_DEFAULT = (1 << 3), //!< \n
    CONTRAST_PLUS_1  = (1 << 4), //!< \n
    CONTRAST_PLUS_2  = (1 << 5), //!< \n
};

//! Camera Shot mode
enum SHOT_MODE {
    SHOT_MODE_SINGLE        = 0, //!<
    SHOT_MODE_CONTINUOUS    = 1, //!<
    SHOT_MODE_PANORAMA      = 2, //!<
    SHOT_MODE_SMILE         = 3, //!<
    SHOT_MODE_SELF          = 6, //!<
};

//! ISO modes
enum ISO {
    ISO_AUTO        = (1 << 0), //!< \n
    ISO_100         = (1 << 1), //!< \n
    ISO_200         = (1 << 2), //!< \n
    ISO_400         = (1 << 3), //!< \n
    ISO_800         = (1 << 4), //!< \n
    ISO_1600        = (1 << 5), //!< \n
};

typedef struct resolution {
    int width;
    int height;
} resolution_t;

typedef struct fps_range {
    int min;
    int max;
} fps_range_t;

class ExynosCameraInfo
{
public:
    camera_info_t cameraInfo;

    resolution_t       *previewSizeValues;
    int                 previewSizeCount;
    resolution_t        previewSize;
    int                 previewFormatValues;
    int                 previewFormat;
    int                *previewFpsValues;
    int                 previewFpsCount;
    int                 previewFps;
    fps_range_t        *previewFpsRangeValues;
    int                 previewFpsRangeCount;
    fps_range_t         previewFpsRange;

    resolution_t       *pictureSizeValues;
    int                 pictureSizeCount;
    resolution_t        pictureSize;
    int                 pictureFormatValues;
    int                 pictureFormat;
    int                 pictureQuality;
    resolution_t       *thumbnailSizeValues;
    int                 thumbnailSizeCount;
    resolution_t        thumbnailSize;
    int                 thumbnailQuality;

    resolution_t       *videoSizeValues;
    int                 videoSizeCount;
    resolution_t        videoSize;
    resolution_t        videoPreferredPreviewSize;
    int                 videoFormatValues;
    int                 videoFormat;
    bool                videoStabilizationSupported;
    bool                videoStabilization;
    bool                videoSnapshotSupported;

    int                 focusModeValues;
    int                 focusMode;
    int                 focusDistances;
    int                 focusMaxAreas;

    bool                zoomSupported;
    bool                zoomSmoothSupported;
    int                *zoomRatioValues;
    int                 zoomRatioCount;
    int                 zoomMax;
    int                 zoom;

    int                 flashModeValues;
    int                 flashMode;

    int                 exposureCompensationMin;
    int                 exposureCompensationMax;
    int                 exposureCompensation;
    float               exposureCompensationStep;

    int                 antiBandingValues;
    int                 antiBanding;

    int                 whiteBalanceValues;
    int                 whiteBalance;
    bool                autoWhiteBalanceLockSupported;
    bool                autoWhiteBalanceLock;

    int                 sceneModeValues;
    int                 sceneMode;

    int                 effectValues;
    int                 effect;

    int                 faceDetectionHwMax;
    int                 faceDetectionSwMax;

    int                 rotation;
    float               viewAngleHorizontal;
    float               viewAngleVertical;

    float               focalLength;

    int                 meteringMaxAreas;

    int                 iso;

    int                 contrast;

    int                 brightnessMax;
    int                 brightnessMin;
    int                 brightness;

    int                 hueMax;
    int                 hueMin;
    int                 hue;

    int                 saturationMax;
    int                 saturationMin;
    int                 saturation;

    int                 sharpnessMax;
    int                 sharpnessMin;
    int                 sharpness;

    int                 vtmode;
    int                 wdr;

public:
    ExynosCameraInfo();
    virtual ~ExynosCameraInfo();

    virtual void                toParameters(CameraParameters *p);
    virtual void                fromParameters(const CameraParameters *p);
};

class ExynosCameraInfoIMX135 : public ExynosCameraInfo
{
public:
    ExynosCameraInfoIMX135();

private:
};

class ExynosCameraInfoS5K6B2 : public ExynosCameraInfo
{
public:
    ExynosCameraInfoS5K6B2();
};

}; // namespace android

#endif /* _EXYNOS_CAMERA_INFO_H_ */

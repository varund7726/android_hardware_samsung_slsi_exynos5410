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

//! Jpeg Qualtiy
enum JPEG_QUALITY {
    JPEG_QUALITY_MIN        = 0,    //!<
    JPEG_QUALITY_ECONOMY    = 70,   //!<
    JPEG_QUALITY_NORMAL     = 80,   //!<
    JPEG_QUALITY_SUPERFINE  = 90,   //!<
    JPEG_QUALITY_MAX        = 100,  //!<
};

struct ExynosCameraInfo
{
public:
    // HAL camera info
    camera_info_t cameraInfo;

    // Google Official API : Camera.Parameters
    // http://developer.android.com/reference/android/hardware/Camera.Parameters.html
    int  previewW;
    int  previewH;
    int  previewColorFormat;
    int  videoW;
    int  videoH;
    int  videoColorFormat;
    int  pictureW;
    int  pictureH;
    int  pictureColorFormat;
    int  thumbnailW;
    int  thumbnailH;

    int  antiBandingList;
    int  antiBanding;

    int  effectList;
    int  effect;

    int  flashModeList;
    int  flashMode;

    int  focusModeList;
    int  focusMode;

    int  sceneModeList;
    int  sceneMode;

    int  whiteBalanceList;
    int  whiteBalance;
    bool autoWhiteBalanceLockSupported;
    bool autoWhiteBalanceLock;

    int  rotation;
    int  minExposure;
    int  maxExposure;
    int  exposure;

    bool autoExposureLockSupported;
    bool autoExposureLock;

    int  fps;
    int  focalLengthNum;
    int  focalLengthDen;
    bool supportVideoStabilization;
    bool applyVideoStabilization;
    bool videoStabilization;
    int  maxNumMeteringAreas;
    int  maxNumDetectedFaces;
    int  maxNumFocusAreas;
    int  maxZoom;
    bool hwZoomSupported;
    int  zoom;

    long gpsLatitude;
    long gpsLongitude;
    long gpsAltitude;
    long gpsTimestamp;

    // Additional API.
    int  angle;
    bool antiShake;
    bool beautyShot;
    int  brightness;
    int  contrast;
    bool gamma;
    bool odc;
    int  hue;
    int  iso;
    int  metering;
    bool objectTracking;
    bool objectTrackingStart;

    int  saturation;
    int  sharpness;
    int  shotMode;
    bool slowAE;
    bool smartAuto;
    bool touchAfStart;
    bool wdr;
    bool tdnr;

public:
    ExynosCameraInfo();
};

struct ExynosCameraInfoIMX135 : public ExynosCameraInfo
{
public:
    ExynosCameraInfoIMX135();
}

struct ExynosCameraInfoS5K6B2 : public ExynosCameraInfo
{
public:
    ExynosCameraInfoS5K6B2();
}

}; // namespace android

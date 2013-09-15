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

#include "ExynosCameraInfo.h"

#define LOG_TAG "ExynosCameraInfo"

using namespace android;

namespace android {


ExynosCameraInfo::ExynosCameraInfo()
{
    cameraInfo.facing = CAMERA_FACING_BACK;
    cameraInfo.orientation = 0;

    previewW = 2560;
    previewH = 1920;
    previewColorFormat = V4L2_PIX_FMT_NV21;
    videoW = 1920;
    videoH = 1080;
    prefVideoPreviewW = 640;
    prefVideoPreviewH = 360;
    videoColorFormat = V4L2_PIX_FMT_NV12M;
    pictureW = 2560;
    pictureH = 1920;
    pictureColorFormat = V4L2_PIX_FMT_YUYV;
    thumbnailW = 320;
    thumbnailH = 240;

    antiBandingList =
          ANTIBANDING_OFF
        | ANTIBANDING_50HZ
        | ANTIBANDING_60HZ
        | ANTIBANDING_OFF;
    antiBanding = ANTIBANDING_OFF;

    effectList =
          EFFECT_NONE
        | EFFECT_MONO
        | EFFECT_NEGATIVE
        | EFFECT_SOLARIZE
        | EFFECT_SEPIA
        | EFFECT_POSTERIZE
        | EFFECT_WHITEBOARD
        | EFFECT_BLACKBOARD
        | EFFECT_AQUA;
    effect = EFFECT_NONE;

    flashModeList =
          FLASH_MODE_OFF
        | FLASH_MODE_AUTO
        | FLASH_MODE_ON
        | FLASH_MODE_RED_EYE
        | FLASH_MODE_TORCH;
    flashMode = FLASH_MODE_OFF;

    focusModeList =
          FOCUS_MODE_AUTO
        | FOCUS_MODE_INFINITY
        | FOCUS_MODE_MACRO
        | FOCUS_MODE_FIXED
        | FOCUS_MODE_EDOF
        | FOCUS_MODE_CONTINUOUS_VIDEO
        | FOCUS_MODE_CONTINUOUS_PICTURE
        | FOCUS_MODE_TOUCH;
    focusMode = FOCUS_MODE_AUTO;

    sceneModeList =
          SCENE_MODE_AUTO
        | SCENE_MODE_ACTION
        | SCENE_MODE_PORTRAIT
        | SCENE_MODE_LANDSCAPE
        | SCENE_MODE_NIGHT
        | SCENE_MODE_NIGHT_PORTRAIT
        | SCENE_MODE_THEATRE
        | SCENE_MODE_BEACH
        | SCENE_MODE_SNOW
        | SCENE_MODE_SUNSET
        | SCENE_MODE_STEADYPHOTO
        | SCENE_MODE_FIREWORKS
        | SCENE_MODE_SPORTS
        | SCENE_MODE_PARTY
        | SCENE_MODE_CANDLELIGHT;
    sceneMode = SCENE_MODE_AUTO;

    whiteBalanceList =
          WHITE_BALANCE_AUTO
        | WHITE_BALANCE_INCANDESCENT
        | WHITE_BALANCE_FLUORESCENT
        | WHITE_BALANCE_WARM_FLUORESCENT
        | WHITE_BALANCE_DAYLIGHT
        | WHITE_BALANCE_CLOUDY_DAYLIGHT
        | WHITE_BALANCE_TWILIGHT
        | WHITE_BALANCE_SHADE;
    whiteBalance = WHITE_BALANCE_AUTO;

    autoWhiteBalanceLockSupported = false;
    autoWhiteBalanceLock = false;

    rotation = 0;
    minExposure = -2;
    maxExposure = 2;
    exposure = 0;

    autoExposureLockSupported = false;
    autoExposureLock = false;

    fps = 30;
    focalLengthNum = 9;
    focalLengthDen = 10;
    supportVideoStabilization = false;
    applyVideoStabilization = false;
    videoStabilization = false;
    maxNumMeteringAreas = 0;
    maxNumDetectedFaces = 0;
    maxNumFocusAreas = 0;
    maxZoom = ZOOM_LEVEL_MAX;
    hwZoomSupported = false;
    zoom = 0;
    gpsAltitude = 0;
    gpsLatitude = 0;
    gpsLongitude = 0;
    gpsTimestamp = 0;

    // Additional API default Value.
    angle = 0;
    antiShake = false;
    beautyShot = false;
    brightness = 0;
    contrast = CONTRAST_DEFAULT;
    gamma = false;
    hue = 2; // 2 is default;
    iso = 0;
    metering = METERING_MODE_CENTER;
    objectTracking = false;
    objectTrackingStart = false;
    saturation = 0;
    sharpness = 0;
    shotMode = SHOT_MODE_SINGLE;
    slowAE = false;
    smartAuto = false;
    touchAfStart = false;
    wdr = false;
    tdnr = false;
    odc = false;
}

ExynosCameraInfoIMX135::ExynosCameraInfoIMX135()
{
    cameraInfo.facing = CAMERA_FACING_BACK;
    cameraInfo.orientation = 0;

    previewW = 1920;
    previewH = 1080;
    previewColorFormat = V4L2_PIX_FMT_NV21;
    videoW = 1920;
    videoH = 1080;
    prefVideoPreviewW = 1920;
    prefVideoPreviewH = 1080;
    videoColorFormat = V4L2_PIX_FMT_NV12M;
    pictureW = 4128;
    pictureH = 2322;
    pictureColorFormat = V4L2_PIX_FMT_YUYV;
    thumbnailW = 512;
    thumbnailH = 384;

    focalLengthNum = 420;
    focalLengthDen = 100;

    antiBandingList =
          ANTIBANDING_OFF
        | ANTIBANDING_50HZ
        | ANTIBANDING_60HZ
        | ANTIBANDING_OFF;
    antiBanding = ANTIBANDING_OFF;

    effectList =
          EFFECT_NONE
        | EFFECT_MONO
        | EFFECT_NEGATIVE
        //| EFFECT_SOLARIZE
        | EFFECT_SEPIA
        //| EFFECT_POSTERIZE
        //| EFFECT_WHITEBOARD
        //| EFFECT_BLACKBOARD
        //| EFFECT_AQUA
        ;
    effect = EFFECT_NONE;

    flashModeList =
          FLASH_MODE_OFF
        | FLASH_MODE_AUTO
        | FLASH_MODE_ON
        //| FLASH_MODE_RED_EYE
        | FLASH_MODE_TORCH;
    flashMode = FLASH_MODE_OFF;

    focusModeList =
          FOCUS_MODE_AUTO
        | FOCUS_MODE_INFINITY
        | FOCUS_MODE_MACRO
        //| FOCUS_MODE_FIXED
        //| FOCUS_MODE_EDOF
        | FOCUS_MODE_CONTINUOUS_VIDEO
        | FOCUS_MODE_CONTINUOUS_PICTURE
        | FOCUS_MODE_TOUCH
        ;
    focusMode = FOCUS_MODE_CONTINUOUS_PICTURE;

    sceneModeList =
          SCENE_MODE_AUTO
        //| SCENE_MODE_ACTION
        | SCENE_MODE_PORTRAIT
        | SCENE_MODE_LANDSCAPE
        | SCENE_MODE_NIGHT
        //| SCENE_MODE_NIGHT_PORTRAIT
        //| SCENE_MODE_THEATRE
        | SCENE_MODE_BEACH
        | SCENE_MODE_SNOW
        | SCENE_MODE_SUNSET
        //| SCENE_MODE_STEADYPHOTO
        | SCENE_MODE_FIREWORKS
        | SCENE_MODE_SPORTS
        | SCENE_MODE_PARTY
        | SCENE_MODE_CANDLELIGHT;
    sceneMode = SCENE_MODE_AUTO;

    whiteBalanceList =
          WHITE_BALANCE_AUTO
        | WHITE_BALANCE_INCANDESCENT
        | WHITE_BALANCE_FLUORESCENT
        //| WHITE_BALANCE_WARM_FLUORESCENT
        | WHITE_BALANCE_DAYLIGHT
        | WHITE_BALANCE_CLOUDY_DAYLIGHT
        //| WHITE_BALANCE_TWILIGHT
        //| WHITE_BALANCE_SHADE
        ;
    whiteBalance = WHITE_BALANCE_AUTO;

    autoWhiteBalanceLockSupported = true;
    autoWhiteBalanceLock = false;

}

ExynosCameraInfoS5K6B2::ExynosCameraInfoS5K6B2()
{
    cameraInfo.facing = CAMERA_FACING_FRONT;
    cameraInfo.orientation = 0;

    previewW = 1920;
    previewH = 1080;
    previewColorFormat = V4L2_PIX_FMT_NV21;
    videoW = 1920;
    videoH = 1080;
    prefVideoPreviewW = 1920;
    prefVideoPreviewH = 1080;
    videoColorFormat = V4L2_PIX_FMT_NV12M;
    pictureW = 1920;
    pictureH = 1080;
    pictureColorFormat = V4L2_PIX_FMT_YUYV;
    thumbnailW = 512;
    thumbnailH = 384;

    focalLengthNum = 185;
    focalLengthDen = 100;

    antiBandingList =
          ANTIBANDING_OFF
        //| ANTIBANDING_50HZ
        //| ANTIBANDING_60HZ
        //| ANTIBANDING_OFF
        ;
    antiBanding = ANTIBANDING_OFF;

    effectList =
          EFFECT_NONE
        | EFFECT_MONO
        | EFFECT_NEGATIVE
        //| EFFECT_SOLARIZE
        | EFFECT_SEPIA
        //| EFFECT_POSTERIZE
        //| EFFECT_WHITEBOARD
        //| EFFECT_BLACKBOARD
        //| EFFECT_AQUA
        ;
    effect = EFFECT_NONE;

    flashModeList =
          FLASH_MODE_OFF
        //| FLASH_MODE_AUTO
        //| FLASH_MODE_ON
        //| FLASH_MODE_RED_EYE
        //| FLASH_MODE_TORCH
        ;
    flashMode = FLASH_MODE_OFF;

    focusModeList =
          FOCUS_MODE_AUTO
        | FOCUS_MODE_INFINITY
        | FOCUS_MODE_MACRO
        //| FOCUS_MODE_FIXED
        //| FOCUS_MODE_EDOF
        | FOCUS_MODE_CONTINUOUS_VIDEO
        | FOCUS_MODE_CONTINUOUS_PICTURE
        | FOCUS_MODE_TOUCH
        ;
    focusMode = FOCUS_MODE_CONTINUOUS_PICTURE;

    sceneModeList =
          SCENE_MODE_AUTO
        //| SCENE_MODE_ACTION
        | SCENE_MODE_PORTRAIT
        | SCENE_MODE_LANDSCAPE
        | SCENE_MODE_NIGHT
        //| SCENE_MODE_NIGHT_PORTRAIT
        //| SCENE_MODE_THEATRE
        | SCENE_MODE_BEACH
        | SCENE_MODE_SNOW
        | SCENE_MODE_SUNSET
        //| SCENE_MODE_STEADYPHOTO
        | SCENE_MODE_FIREWORKS
        | SCENE_MODE_SPORTS
        | SCENE_MODE_PARTY
        | SCENE_MODE_CANDLELIGHT;
    sceneMode = SCENE_MODE_AUTO;

    whiteBalanceList =
          WHITE_BALANCE_AUTO
        | WHITE_BALANCE_INCANDESCENT
        | WHITE_BALANCE_FLUORESCENT
        //| WHITE_BALANCE_WARM_FLUORESCENT
        | WHITE_BALANCE_DAYLIGHT
        | WHITE_BALANCE_CLOUDY_DAYLIGHT
        //| WHITE_BALANCE_TWILIGHT
        //| WHITE_BALANCE_SHADE
        ;
    whiteBalance = WHITE_BALANCE_AUTO;

}

}; // namespace android

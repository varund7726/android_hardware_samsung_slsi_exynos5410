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

#include "videodev2.h"
#include "videodev2_exynos_media.h"
#include "videodev2_exynos_camera.h"

#define LOG_TAG "ExynosCameraInfo"

namespace android {

#define ARRAY_SIZE(x) (sizeof((x)) / sizeof((x)[0]))

static resolution_t imx135_previewTable[] = {
    {1920, 1080}, {1440, 1080}, {1280, 960}, {1270, 720},
    {1056, 864}, {1024, 768}, {1008, 566}, {800, 600},
    {800, 480}, {800, 450}, {720, 480}, {640, 480},
    {528, 432}, {480, 320}, {480, 270}, {352, 288},
    {320, 240}, {176, 144}
};

static resolution_t imx135_thumbnailTable[] = {
    {512, 384},
    {512, 288},
    {480, 320},
    {352, 288},
    {320, 240},
    {320, 180}
};

static resolution_t imx135_pictureTable[] = {
    {4128, 3096}, {4128, 2322}, {4096, 3072}, {4096, 2304},
    {3264, 2448}, {3264, 1836}, {3200, 2400}, {3072, 1728},
    {2592, 1936}, {2560, 1920}, {2048, 1536}, {2048, 1152},
    {2034, 1142}, {1920, 1080}, {1600, 1200}, {1440, 1080},
    {1392, 1392}, {1280, 960}, {1280, 720}, {1024, 768},
    {1008, 566}, {800, 600}, {800, 480}, {800, 450},
    {720, 480}, {640, 480}, {528, 432}, {480, 320},
    {480, 270}, {352, 288}, {320, 240}, {320, 180}
};

static resolution_t imx135_videoTable[] = {
    {1920, 1080}, {1440, 1080}, {1280, 720}, {960, 720},
    {800, 450}, {720, 480}, {640, 480}, {480, 320},
    {352, 288}, {320, 240}, {176, 144}
};

static int imx135_zoomRatioTable[] = {
    100, 110, 120, 130, 140, 150, 160, 170, 180, 190,
    200, 210, 220, 230, 240, 250, 260, 270, 280, 290,
    300, 310, 320, 330, 340, 350, 360, 370, 380, 390,
    400
};

void resolution_to_string8(const resolution_t *values, int size, String8 *str)
{
    char strBuf[256];
    const resolution_t *res;

    str->clear();
    for (int i = 0; i < size; i++) {
        res = &values[i];
        snprintf(strBuf, 256, "%dx%d", res->width, res->height);
        if (i > 0)
            str->append(",");
        str->append(strBuf);
    }
}

void format_to_string8(const int format, String8 *str)
{
    str->clear();

    /* kind of ugly hacks for setting correct formats */
    if ((format & V4L2_PIX_FMT_NV21) == V4L2_PIX_FMT_NV21) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::PIXEL_FORMAT_YUV420SP);
    }

    if ((format & V4L2_PIX_FMT_NV12M) == V4L2_PIX_FMT_NV12M) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::PIXEL_FORMAT_YUV420SP);
    }
}

void fps_range_to_string8(const fps_range_t *values, int size, String8 *str)
{
    char strBuf[256];
    const fps_range_t *range;

    str->clear();
    for (int i = 0; i < size; i++) {
        range = &values[i];
        if (size == 1)
            snprintf(strBuf, 256, "%d,%d", range->min, range->max);
        else
            snprintf(strBuf, 256, "(%d,%d)", range->min, range->max);
        if (i > 0)
            str->append(",");
        str->append(strBuf);
    }
}

void integers_to_string8(const int *values, int size, String8 *str)
{
    char strBuf[256];

    str->clear();
    for (int i = 0; i < size; i++) {
        snprintf(strBuf, 256, "%d", values[i]);
        if (i > 0)
            str->append(",");
        str->append(strBuf);
    }
}

void floats_to_string8(const float *values, int size, String8 *str)
{
    char strBuf[256];

    str->clear();
    for (int i = 0; i < size; i++) {
        snprintf(strBuf, 256, "%.2f", values[i]);
        if (i > 0)
            str->append(",");
        str->append(strBuf);
    }
}

void bool_to_string8(const bool value, String8 *str)
{
    if (value)
        str->setTo("true");
    else
        str->setTo("false");
}

void focus_mode_to_string8(const int modes, String8 *str)
{
    str->clear();

    if (modes & FOCUS_MODE_AUTO) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::FOCUS_MODE_AUTO);
    }
    if (modes & FOCUS_MODE_INFINITY) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::FOCUS_MODE_INFINITY);
    }
    if (modes & FOCUS_MODE_MACRO) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::FOCUS_MODE_MACRO);
    }
    if (modes & FOCUS_MODE_FIXED) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::FOCUS_MODE_FIXED);
    }
    if (modes & FOCUS_MODE_EDOF) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::FOCUS_MODE_EDOF);
    }
    if (modes & FOCUS_MODE_CONTINUOUS_VIDEO) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::FOCUS_MODE_CONTINUOUS_VIDEO);
    }
    if (modes & FOCUS_MODE_CONTINUOUS_PICTURE) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::FOCUS_MODE_CONTINUOUS_PICTURE);
    }
}

int string_to_focus_mode(const char *mode)
{
    int ret = 0;

    if (strcmp(mode, CameraParameters::FOCUS_MODE_AUTO) == 0) {
        ret = FOCUS_MODE_AUTO;
    } else if (strcmp(mode, CameraParameters::FOCUS_MODE_INFINITY) == 0) {
        ret = FOCUS_MODE_INFINITY;
    } else if (strcmp(mode, CameraParameters::FOCUS_MODE_MACRO) == 0) {
        ret = FOCUS_MODE_MACRO;
    } else if (strcmp(mode, CameraParameters::FOCUS_MODE_FIXED) == 0) {
        ret = FOCUS_MODE_FIXED;
    } else if (strcmp(mode, CameraParameters::FOCUS_MODE_EDOF) == 0) {
        ret = FOCUS_MODE_EDOF;
    } else if (strcmp(mode,
                CameraParameters::FOCUS_MODE_CONTINUOUS_VIDEO) == 0) {
        ret = FOCUS_MODE_CONTINUOUS_VIDEO;
    } else if (strcmp(mode,
                CameraParameters::FOCUS_MODE_CONTINUOUS_PICTURE) == 0) {
        ret = FOCUS_MODE_CONTINUOUS_PICTURE;
    } else {
        ALOGW("WARN(%s): Unable to find matching focus mode %s, default auto",
                __FUNCTION__, mode);
        ret = FOCUS_MODE_AUTO;
    }

    return ret;
}

void focus_distance_to_string8(const int distances, String8 *str)
{
    str->clear();

    if (distances & FOCUS_DISTANCE_10) {
        if (str->length() > 0)
            str->append(",");
        str->append("0.10");
    }
    if (distances & FOCUS_DISTANCE_120) {
        if (str->length() > 0)
            str->append(",");
        str->append("1.20");
    }
    if (distances & FOCUS_DISTANCE_INF) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::FOCUS_DISTANCE_INFINITY);
    }
}

void flash_mode_to_string8(const int modes, String8 *str)
{
    str->clear();

    if (modes & FLASH_MODE_OFF) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::FLASH_MODE_OFF);
    }
    if (modes & FLASH_MODE_AUTO) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::FLASH_MODE_AUTO);
    }
    if (modes & FLASH_MODE_ON) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::FLASH_MODE_ON);
    }
    if (modes & FLASH_MODE_RED_EYE) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::FLASH_MODE_RED_EYE);
    }
    if (modes & FLASH_MODE_TORCH) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::FLASH_MODE_TORCH);
    }
}

int string_to_flash_mode(const char *mode)
{
    int ret = 0;

    if (strcmp(mode, CameraParameters::FLASH_MODE_OFF) == 0) {
        ret = FLASH_MODE_OFF;
    } else if (strcmp(mode, CameraParameters::FLASH_MODE_AUTO) == 0) {
        ret = FLASH_MODE_AUTO;
    } else if (strcmp(mode, CameraParameters::FLASH_MODE_ON) == 0) {
        ret = FLASH_MODE_ON;
    } else if (strcmp(mode, CameraParameters::FLASH_MODE_RED_EYE) == 0) {
        ret = FLASH_MODE_RED_EYE;
    } else if (strcmp(mode, CameraParameters::FLASH_MODE_TORCH) == 0) {
        ret = FLASH_MODE_TORCH;
    } else {
        ALOGW("WARN(%s): Unable to find matching flash mode %s, default off",
                __FUNCTION__, mode);
        ret = FLASH_MODE_OFF;
    }

    return ret;
}

void antibanding_to_string8(const int modes, String8 *str)
{
    str->clear();

    if (modes & ANTIBANDING_AUTO) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::ANTIBANDING_AUTO);
    }
    if (modes & ANTIBANDING_50HZ) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::ANTIBANDING_50HZ);
    }
    if (modes & ANTIBANDING_60HZ) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::ANTIBANDING_60HZ);
    }
    if (modes & ANTIBANDING_OFF) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::ANTIBANDING_OFF);
    }
}

int string_to_antibanding(const char *mode)
{
    int ret = 0;

    if (strcmp(mode, CameraParameters::ANTIBANDING_AUTO) == 0) {
        ret = ANTIBANDING_AUTO;
    } else if (strcmp(mode, CameraParameters::ANTIBANDING_50HZ) == 0) {
        ret = ANTIBANDING_50HZ;
    } else if (strcmp(mode, CameraParameters::ANTIBANDING_60HZ) == 0) {
        ret = ANTIBANDING_60HZ;
    } else if (strcmp(mode, CameraParameters::ANTIBANDING_OFF) == 0) {
        ret = ANTIBANDING_OFF;
    } else {
        ALOGW("WARN(%s): Unable to find matching antibanding mode %s, default off",
                __FUNCTION__, mode);
        ret = ANTIBANDING_OFF;
    }

    return ret;
}

void white_balance_to_string8(const int modes, String8 *str) {
    str->clear();

    if (modes & WHITE_BALANCE_AUTO) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::WHITE_BALANCE_AUTO);
    }
    if (modes & WHITE_BALANCE_INCANDESCENT) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::WHITE_BALANCE_INCANDESCENT);
    }
    if (modes & WHITE_BALANCE_FLUORESCENT) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::WHITE_BALANCE_FLUORESCENT);
    }
    if (modes & WHITE_BALANCE_WARM_FLUORESCENT) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::WHITE_BALANCE_WARM_FLUORESCENT);
    }
    if (modes & WHITE_BALANCE_DAYLIGHT) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::WHITE_BALANCE_DAYLIGHT);
    }
    if (modes & WHITE_BALANCE_CLOUDY_DAYLIGHT) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::WHITE_BALANCE_CLOUDY_DAYLIGHT);
    }
    if (modes & WHITE_BALANCE_TWILIGHT) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::WHITE_BALANCE_TWILIGHT);
    }
    if (modes & WHITE_BALANCE_SHADE) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::WHITE_BALANCE_SHADE);
    }
}

int string_to_white_balance(const char *mode)
{
    int ret = 0;

    if (strcmp(mode, CameraParameters::WHITE_BALANCE_AUTO) == 0) {
        ret = WHITE_BALANCE_AUTO;
    } else if (strcmp(mode, CameraParameters::WHITE_BALANCE_INCANDESCENT) == 0) {
        ret = WHITE_BALANCE_INCANDESCENT;
    } else if (strcmp(mode, CameraParameters::WHITE_BALANCE_FLUORESCENT) == 0) {
        ret = WHITE_BALANCE_FLUORESCENT;
    } else if (strcmp(mode, CameraParameters::WHITE_BALANCE_WARM_FLUORESCENT) == 0) {
        ret = WHITE_BALANCE_WARM_FLUORESCENT;
    } else if (strcmp(mode, CameraParameters::WHITE_BALANCE_DAYLIGHT) == 0) {
        ret = WHITE_BALANCE_DAYLIGHT;
    } else if (strcmp(mode, CameraParameters::WHITE_BALANCE_CLOUDY_DAYLIGHT) == 0) {
        ret = WHITE_BALANCE_CLOUDY_DAYLIGHT;
    } else if (strcmp(mode, CameraParameters::WHITE_BALANCE_TWILIGHT) == 0) {
        ret = WHITE_BALANCE_TWILIGHT;
    } else if (strcmp(mode, CameraParameters::WHITE_BALANCE_SHADE) == 0) {
        ret = WHITE_BALANCE_SHADE;
    } else {
        ALOGW("WARN(%s): Unable to find matching white balance mode %s, default auto",
                __FUNCTION__, mode);
        ret = WHITE_BALANCE_AUTO;
    }

    return ret;
}

void scene_to_string8(const int modes, String8 *str)
{
    str->clear();

    if (modes & SCENE_MODE_AUTO) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::SCENE_MODE_AUTO);
    }
    if (modes & SCENE_MODE_ACTION) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::SCENE_MODE_ACTION);
    }
    if (modes & SCENE_MODE_PORTRAIT) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::SCENE_MODE_PORTRAIT);
    }
    if (modes & SCENE_MODE_LANDSCAPE) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::SCENE_MODE_LANDSCAPE);
    }
    if (modes & SCENE_MODE_NIGHT) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::SCENE_MODE_NIGHT);
    }
    if (modes & SCENE_MODE_NIGHT_PORTRAIT) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::SCENE_MODE_NIGHT_PORTRAIT);
    }
    if (modes & SCENE_MODE_THEATRE) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::SCENE_MODE_THEATRE);
    }
    if (modes & SCENE_MODE_BEACH) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::SCENE_MODE_BEACH);
    }
    if (modes & SCENE_MODE_SNOW) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::SCENE_MODE_SNOW);
    }
    if (modes & SCENE_MODE_SUNSET) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::SCENE_MODE_SUNSET);
    }
    if (modes & SCENE_MODE_STEADYPHOTO) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::SCENE_MODE_STEADYPHOTO);
    }
    if (modes & SCENE_MODE_FIREWORKS) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::SCENE_MODE_FIREWORKS);
    }
    if (modes & SCENE_MODE_SPORTS) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::SCENE_MODE_SPORTS);
    }
    if (modes & SCENE_MODE_PARTY) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::SCENE_MODE_PARTY);
    }
    if (modes & SCENE_MODE_CANDLELIGHT) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::SCENE_MODE_CANDLELIGHT);
    }
}

int string_to_scene_mode(const char *mode)
{
    int ret = 0;

    if (strcmp(mode, CameraParameters::SCENE_MODE_AUTO) == 0) {
        ret = SCENE_MODE_AUTO;
    } else if (strcmp(mode, CameraParameters::SCENE_MODE_ACTION) == 0) {
        ret = SCENE_MODE_ACTION;
    } else if (strcmp(mode, CameraParameters::SCENE_MODE_PORTRAIT) == 0) {
        ret = SCENE_MODE_PORTRAIT;
    } else if (strcmp(mode, CameraParameters::SCENE_MODE_LANDSCAPE) == 0) {
        ret = SCENE_MODE_LANDSCAPE;
    } else if (strcmp(mode, CameraParameters::SCENE_MODE_NIGHT) == 0) {
        ret = SCENE_MODE_NIGHT;
    } else if (strcmp(mode, CameraParameters::SCENE_MODE_NIGHT_PORTRAIT) == 0) {
        ret = SCENE_MODE_NIGHT_PORTRAIT;
    } else if (strcmp(mode, CameraParameters::SCENE_MODE_THEATRE) == 0) {
        ret = SCENE_MODE_THEATRE;
    } else if (strcmp(mode, CameraParameters::SCENE_MODE_BEACH) == 0) {
        ret = SCENE_MODE_BEACH;
    } else if (strcmp(mode, CameraParameters::SCENE_MODE_SNOW) == 0) {
        ret = SCENE_MODE_SNOW;
    } else if (strcmp(mode, CameraParameters::SCENE_MODE_SUNSET) == 0) {
        ret = SCENE_MODE_SUNSET;
    } else if (strcmp(mode, CameraParameters::SCENE_MODE_STEADYPHOTO) == 0) {
        ret = SCENE_MODE_STEADYPHOTO;
    } else if (strcmp(mode, CameraParameters::SCENE_MODE_FIREWORKS) == 0) {
        ret = SCENE_MODE_FIREWORKS;
    } else if (strcmp(mode, CameraParameters::SCENE_MODE_SPORTS) == 0) {
        ret = SCENE_MODE_SPORTS;
    } else if (strcmp(mode, CameraParameters::SCENE_MODE_PARTY) == 0) {
        ret = SCENE_MODE_PARTY;
    } else if (strcmp(mode, CameraParameters::SCENE_MODE_CANDLELIGHT) == 0) {
        ret = SCENE_MODE_CANDLELIGHT;
    } else {
        ALOGW("WARN(%s): Unable to find matching scene mode %s, default auto",
                __FUNCTION__, mode);
        ret = SCENE_MODE_AUTO;
    }

    return ret;
}

void effect_to_string8(const int modes, String8 *str)
{
    str->clear();
    if (modes & EFFECT_NONE) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::EFFECT_NONE);
    }
    if (modes & EFFECT_MONO) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::EFFECT_MONO);
    }
    if (modes & EFFECT_NEGATIVE) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::EFFECT_NEGATIVE);
    }
    if (modes & EFFECT_SOLARIZE) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::EFFECT_SOLARIZE);
    }
    if (modes & EFFECT_SEPIA) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::EFFECT_SEPIA);
    }
    if (modes & EFFECT_POSTERIZE) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::EFFECT_POSTERIZE);
    }
    if (modes & EFFECT_WHITEBOARD) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::EFFECT_WHITEBOARD);
    }
    if (modes & EFFECT_BLACKBOARD) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::EFFECT_BLACKBOARD);
    }
    if (modes & EFFECT_AQUA) {
        if (str->length() > 0)
            str->append(",");
        str->append(CameraParameters::EFFECT_AQUA);
    }
}

int string_to_effect(const char *mode)
{
    int ret = 0;

    if (strcmp(mode, CameraParameters::EFFECT_NONE) == 0) {
        ret = EFFECT_NONE;
    } else if (strcmp(mode, CameraParameters::EFFECT_MONO) == 0) {
        ret = EFFECT_MONO;
    } else if (strcmp(mode, CameraParameters::EFFECT_NEGATIVE) == 0) {
        ret = EFFECT_NEGATIVE;
    } else if (strcmp(mode, CameraParameters::EFFECT_SOLARIZE) == 0) {
        ret = EFFECT_SOLARIZE;
    } else if (strcmp(mode, CameraParameters::EFFECT_SEPIA) == 0) {
        ret = EFFECT_SEPIA;
    } else if (strcmp(mode, CameraParameters::EFFECT_POSTERIZE) == 0) {
        ret = EFFECT_POSTERIZE;
    } else if (strcmp(mode, CameraParameters::EFFECT_WHITEBOARD) == 0) {
        ret = EFFECT_WHITEBOARD;
    } else if (strcmp(mode, CameraParameters::EFFECT_BLACKBOARD) == 0) {
        ret = EFFECT_BLACKBOARD;
    } else if (strcmp(mode, CameraParameters::EFFECT_AQUA) == 0) {
        ret = EFFECT_AQUA;
    } else {
        ALOGW("WARN(%s): Unable to match effect %s, default none",
                __FUNCTION__, mode);
        ret = EFFECT_NONE;
    }

    return ret;
}

void iso_to_string8(const int modes, String8 *str)
{
    str->clear();

    if (modes & ISO_AUTO) {
        if (str->length() > 0)
            str->append(",");
        str->append("auto");
    }
    if (modes & ISO_100) {
        if (str->length() > 0)
            str->append(",");
        str->append("ISO100");
    }
    if (modes & ISO_200) {
        if (str->length() > 0)
            str->append(",");
        str->append("ISO200");
    }
    if (modes & ISO_400) {
        if (str->length() > 0)
            str->append(",");
        str->append("ISO400");
    }
    if (modes & ISO_800) {
        if (str->length() > 0)
            str->append(",");
        str->append("ISO800");
    }
    if (modes & ISO_1600) {
        if (str->length() > 0)
            str->append(",");
        str->append("ISO1600");
    }
}

int string_to_iso(const char *mode)
{
    int ret = 0;

    if (strcmp(mode, "auto") == 0) {
        ret = ISO_AUTO;
    } else if (strcmp(mode, "ISO100") == 0) {
        ret = ISO_100;
    } else if (strcmp(mode, "ISO200") == 0) {
        ret = ISO_200;
    } else if (strcmp(mode, "ISO400") == 0) {
        ret = ISO_400;
    } else if (strcmp(mode, "ISO800") == 0) {
        ret = ISO_800;
    } else if (strcmp(mode, "ISO1600") == 0) {
        ret = ISO_1600;
    } else {
        ALOGW("WARN(%s): Unable to match ISO mode %s, default auto",
                __FUNCTION__, mode);
        ret = ISO_AUTO;
    }

    return ret;
}

void contrast_to_string8(const int modes, String8 *str)
{
    str->clear();

    if (modes & CONTRAST_AUTO) {
        if (str->length() > 0)
            str->append(",");
        str->append("auto");
    }
    if (modes & CONTRAST_MINUS_2) {
        if (str->length() > 0)
            str->append(",");
        str->append("-2");
    }
    if (modes & CONTRAST_MINUS_1) {
        if (str->length() > 0)
            str->append(",");
        str->append("-1");
    }
    if (modes & CONTRAST_DEFAULT) {
        if (str->length() > 0)
            str->append(",");
        str->append("0");
    }
    if (modes & CONTRAST_PLUS_1) {
        if (str->length() > 0)
            str->append(",");
        str->append("1");
    }
    if (modes & CONTRAST_PLUS_2) {
        if (str->length() > 0)
            str->append(",");
        str->append("2");
    }
}

int string_to_contrast(const char *mode)
{
    int ret = 0;

    if (strcmp(mode, "auto") == 0) {
        ret = CONTRAST_AUTO;
    } else if (strcmp(mode, "-2") == 0) {
        ret = CONTRAST_MINUS_2;
    } else if (strcmp(mode, "-1") == 0) {
        ret = CONTRAST_MINUS_1;
    } else if (strcmp(mode, "0") == 0) {
        ret = CONTRAST_DEFAULT;
    } else if (strcmp(mode, "1") == 0) {
        ret = CONTRAST_PLUS_1;
    } else if (strcmp(mode, "2") == 0) {
        ret = CONTRAST_PLUS_2;
    } else {
        ALOGW("WARN(%s): Unable to match contrast mode %s, default auto",
                __FUNCTION__, mode);
        ret = CONTRAST_AUTO;
    }

    return ret;
}

bool is_supported_resolution(resolution_t *res, int size, resolution_t *r)
{
    if (res) {
        for (int i = 0; i < size; i++) {
            if (res[i].width == r->width && res[i].height == r->height)
                return true;
        }
    }

    return false;
}

bool is_supported_value(int *vals, int size, int v)
{
    if (vals) {
        for (int i = 0; i < size; i++) {
            if (vals[i] == v)
                return true;
        }
    }

    return false;
}

bool is_supported_fps_range(fps_range_t *ranges, int size, fps_range_t *r)
{
    if (ranges) {
        for (int i = 0; i < size; i++) {
            if (ranges[i].min == r->min && ranges[i].max == r->max)
                return true;
        }
    }

    return false;
}

int string_to_v4l2_format(const char *format)
{
    int ret;

    if (strcmp(format, CameraParameters::PIXEL_FORMAT_RGB565) == 0) {
        ret = V4L2_PIX_FMT_RGB565;
    } else if (strcmp(format, CameraParameters::PIXEL_FORMAT_RGBA8888) == 0) {
        ret = V4L2_PIX_FMT_RGB32;
    } else if (strcmp(format, CameraParameters::PIXEL_FORMAT_YUV420SP) == 0) {
        ret = V4L2_PIX_FMT_NV21;
    } else if (strcmp(format, CameraParameters::PIXEL_FORMAT_YUV420P) == 0) {
        ret = V4L2_PIX_FMT_YUV420;
    } else if (strcmp(format, CameraParameters::PIXEL_FORMAT_JPEG) == 0) {
        ret = V4L2_PIX_FMT_YUYV;
    } else {
        ALOGW("ERR(%s): Unknown format %s, defaulting to NV21",
                __FUNCTION__, format);
        ret = V4L2_PIX_FMT_NV21;
    }

    return ret;
}

bool get_bool(const CameraParameters *p, const char *key)
{
    const char *val = p->get(key);
    if (strcmp(val, "true") == 0)
        return true;
    return false;
}

ExynosCameraInfo::ExynosCameraInfo()
{
}

ExynosCameraInfo::~ExynosCameraInfo()
{
}

void ExynosCameraInfo::toParameters(CameraParameters *p)
{
    String8 paramStr;

    /* Preview */
    resolution_to_string8(previewSizeValues, previewSizeCount, &paramStr);
    p->set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES, paramStr.string());

    p->setPreviewSize(previewSize.width, previewSize.height);

    format_to_string8(previewFormatValues, &paramStr);
    p->set(CameraParameters::KEY_SUPPORTED_PREVIEW_FORMATS, paramStr.string());

    format_to_string8(previewFormat, &paramStr);
    p->setPreviewFormat(paramStr.string());

    integers_to_string8(previewFpsValues, previewFpsCount, &paramStr);
    p->set(CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES, paramStr.string());

    fps_range_to_string8(previewFpsRangeValues, previewFpsRangeCount, &paramStr);
    p->set(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE, paramStr.string());

    fps_range_to_string8(&previewFpsRange, 1, &paramStr);
    p->set(CameraParameters::KEY_PREVIEW_FPS_RANGE, paramStr.string());

    /* Picture */
    resolution_to_string8(pictureSizeValues, pictureSizeCount, &paramStr);
    p->set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES, paramStr.string());

    p->setPictureSize(pictureSize.width, pictureSize.height);

    /* Lock to JPEG format */
    p->set(CameraParameters::KEY_SUPPORTED_PICTURE_FORMATS, CameraParameters::PIXEL_FORMAT_JPEG);
    p->setPictureFormat(CameraParameters::PIXEL_FORMAT_JPEG);

    integers_to_string8(&pictureQuality, 1, &paramStr);
    p->set(CameraParameters::KEY_JPEG_QUALITY, paramStr.string());

    resolution_to_string8(thumbnailSizeValues, thumbnailSizeCount, &paramStr);
    p->set(CameraParameters::KEY_SUPPORTED_JPEG_THUMBNAIL_SIZES, paramStr.string());

    p->set(CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH, thumbnailSize.width);
    p->set(CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT, thumbnailSize.height);

    integers_to_string8(&thumbnailQuality, 1, &paramStr);
    p->set(CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY, paramStr.string());

    /* Video */
    resolution_to_string8(videoSizeValues, videoSizeCount, &paramStr);
    p->set(CameraParameters::KEY_SUPPORTED_VIDEO_SIZES, paramStr.string());

    p->setVideoSize(videoSize.width, videoSize.height);

    resolution_to_string8(&videoPreferredPreviewSize, 1, &paramStr);
    p->set(CameraParameters::KEY_PREFERRED_PREVIEW_SIZE_FOR_VIDEO, paramStr.string());

    format_to_string8(videoFormat, &paramStr);
    p->set(CameraParameters::KEY_VIDEO_FRAME_FORMAT, paramStr.string());

    bool_to_string8(videoStabilizationSupported, &paramStr);
    p->set(CameraParameters::KEY_VIDEO_STABILIZATION_SUPPORTED, paramStr.string());

    bool_to_string8(videoSnapshotSupported, &paramStr);
    p->set(CameraParameters::KEY_VIDEO_SNAPSHOT_SUPPORTED, paramStr.string());

    /* Focus */
    focus_mode_to_string8(focusModeValues, &paramStr);
    p->set(CameraParameters::KEY_SUPPORTED_FOCUS_MODES, paramStr.string());

    focus_mode_to_string8(focusMode, &paramStr);
    p->set(CameraParameters::KEY_FOCUS_MODE, paramStr.string());

    focus_distance_to_string8(focusDistances, &paramStr);
    p->set(CameraParameters::KEY_FOCUS_DISTANCES, paramStr.string());

    p->set(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS, focusMaxAreas);

    /* don't know why this is set */
    p->set(CameraParameters::FOCUS_DISTANCE_INFINITY,
            CameraParameters::FOCUS_DISTANCE_INFINITY);

    /* Zoom */
    bool_to_string8(zoomSupported, &paramStr);
    p->set(CameraParameters::KEY_ZOOM_SUPPORTED, paramStr.string());

    bool_to_string8(zoomSmoothSupported, &paramStr);
    p->set(CameraParameters::KEY_SMOOTH_ZOOM_SUPPORTED, paramStr.string());

    integers_to_string8(zoomRatioValues, zoomRatioCount, &paramStr);
    p->set(CameraParameters::KEY_ZOOM_RATIOS, paramStr.string());

    integers_to_string8(&zoomMax, 1, &paramStr);
    p->set(CameraParameters::KEY_MAX_ZOOM, paramStr.string());

    integers_to_string8(&zoom, 1, &paramStr);
    p->set(CameraParameters::KEY_ZOOM, paramStr.string());

    /* Flash */
    flash_mode_to_string8(flashModeValues, &paramStr);
    p->set(CameraParameters::KEY_SUPPORTED_FLASH_MODES, paramStr.string());

    flash_mode_to_string8(flashMode, &paramStr);
    p->set(CameraParameters::KEY_FLASH_MODE, paramStr.string());

    /* Exposure compensation */
    integers_to_string8(&exposureCompensationMin, 1, &paramStr);
    p->set(CameraParameters::KEY_MIN_EXPOSURE_COMPENSATION, paramStr.string());

    integers_to_string8(&exposureCompensationMax, 1, &paramStr);
    p->set(CameraParameters::KEY_MAX_EXPOSURE_COMPENSATION, paramStr.string());

    integers_to_string8(&exposureCompensation, 1, &paramStr);
    p->set(CameraParameters::KEY_EXPOSURE_COMPENSATION, paramStr.string());

    floats_to_string8(&exposureCompensationStep, 1, &paramStr);
    p->set(CameraParameters::KEY_EXPOSURE_COMPENSATION_STEP, paramStr.string());

    /* Antibanding */
    antibanding_to_string8(antiBandingValues, &paramStr);
    p->set(CameraParameters::KEY_SUPPORTED_ANTIBANDING, paramStr.string());

    antibanding_to_string8(antiBanding, &paramStr);
    p->set(CameraParameters::KEY_ANTIBANDING, paramStr.string());

    /* White balance */
    white_balance_to_string8(whiteBalanceValues, &paramStr);
    p->set(CameraParameters::KEY_SUPPORTED_WHITE_BALANCE, paramStr.string());

    white_balance_to_string8(whiteBalance, &paramStr);
    p->set(CameraParameters::KEY_WHITE_BALANCE, paramStr.string());

    /* Scene */
    scene_to_string8(sceneModeValues, &paramStr);
    p->set(CameraParameters::KEY_SUPPORTED_SCENE_MODES, paramStr.string());

    scene_to_string8(sceneMode, &paramStr);
    p->set(CameraParameters::KEY_SCENE_MODE, paramStr.string());

    /* Effect */
    effect_to_string8(effectValues, &paramStr);
    p->set(CameraParameters::KEY_SUPPORTED_EFFECTS, paramStr.string());

    effect_to_string8(effect, &paramStr);
    p->set(CameraParameters::KEY_EFFECT, paramStr.string());

    /* Face detect */
    integers_to_string8(&faceDetectionHwMax, 1, &paramStr);
    p->set(CameraParameters::KEY_MAX_NUM_DETECTED_FACES_HW, paramStr.string());

    integers_to_string8(&faceDetectionSwMax, 1, &paramStr);
    p->set(CameraParameters::KEY_MAX_NUM_DETECTED_FACES_SW, paramStr.string());

    /* Angles */
    integers_to_string8(&rotation, 1, &paramStr);
    p->set(CameraParameters::KEY_ROTATION, paramStr.string());

    floats_to_string8(&viewAngleHorizontal, 1, &paramStr);
    p->set(CameraParameters::KEY_HORIZONTAL_VIEW_ANGLE, paramStr.string());

    floats_to_string8(&viewAngleVertical, 1, &paramStr);
    p->set(CameraParameters::KEY_VERTICAL_VIEW_ANGLE, paramStr.string());

    /* Misc */
    floats_to_string8(&focalLength, 1, &paramStr);
    p->set(CameraParameters::KEY_FOCAL_LENGTH, paramStr.string());

    integers_to_string8(&meteringMaxAreas, 1, &paramStr);
    p->set(CameraParameters::KEY_MAX_NUM_METERING_AREAS, paramStr.string());

    iso_to_string8(iso, &paramStr);
    p->set("iso", paramStr.string());

    /* Color */
    contrast_to_string8(contrast, &paramStr);
    p->set("contrast", paramStr.string());

    integers_to_string8(&brightnessMax, 1, &paramStr);
    p->set("brightness-max", paramStr.string());

    integers_to_string8(&brightnessMin, 1, &paramStr);
    p->set("brightness-min", paramStr.string());

    integers_to_string8(&brightness, 1, &paramStr);
    p->set("brightness", paramStr.string());

    integers_to_string8(&brightnessMax, 1, &paramStr);
    p->set("hue-max", paramStr.string());

    integers_to_string8(&brightnessMin, 1, &paramStr);
    p->set("hue-min", paramStr.string());

    integers_to_string8(&brightness, 1, &paramStr);
    p->set("hue", paramStr.string());

    integers_to_string8(&brightnessMax, 1, &paramStr);
    p->set("saturation-max", paramStr.string());

    integers_to_string8(&brightnessMin, 1, &paramStr);
    p->set("saturation-min", paramStr.string());

    integers_to_string8(&brightness, 1, &paramStr);
    p->set("saturation", paramStr.string());

    /* Unknown */
    integers_to_string8(&vtmode, 1, &paramStr);
    p->set("vtmode", paramStr.string());

    integers_to_string8(&wdr, 1, &paramStr);
    p->set("wdr", paramStr.string());
}

void ExynosCameraInfo::fromParameters(const CameraParameters *p)
{
    const char *paramStr;

    /* Preview */
    resolution_t newPreviewSize;
    int newPreviewFormat;
    int newPreviewFps;
    fps_range_t newPreviewFpsRange;

    p->getPreviewSize(&newPreviewSize.width, &newPreviewSize.height);
    if (is_supported_resolution(previewSizeValues, previewSizeCount,
                &newPreviewSize)) {
        previewSize = newPreviewSize;
    }

    newPreviewFormat = string_to_v4l2_format(p->getPreviewFormat());
    if (newPreviewFormat & previewFormatValues) {
        previewFormat = newPreviewFormat;
    }

    newPreviewFps = p->getPreviewFrameRate();
    if (is_supported_value(previewFpsValues, previewFpsCount, newPreviewFps)) {
        previewFps = newPreviewFps;
    }

    p->getPreviewFpsRange(&newPreviewFpsRange.min, &newPreviewFpsRange.max);
    if (is_supported_fps_range(previewFpsRangeValues, previewFpsRangeCount,
                &newPreviewFpsRange)) {
        previewFpsRange = newPreviewFpsRange;
    }

    /* Picture */
    resolution_t newPictureSize;
    int newPictureFormat;
    int newPictureQuality;
    resolution_t newThumbnailSize;
    int newThumbnailQuality;

    p->getPictureSize(&newPictureSize.width, &newPictureSize.height);
    if (is_supported_resolution(pictureSizeValues, pictureSizeCount,
                &newPictureSize)) {
        pictureSize = newPictureSize;
    }

    newPictureFormat = string_to_v4l2_format(p->getPictureFormat());
    if (newPictureFormat & pictureFormatValues) {
        pictureFormat = newPictureFormat;
    }

    newPictureQuality = p->getInt(CameraParameters::KEY_JPEG_QUALITY);
    if (newPictureQuality >= 1 && newPictureQuality <= 100) {
        pictureQuality = newPictureQuality;
    }

    newThumbnailSize.width = p->getInt(CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH);
    newThumbnailSize.height = p->getInt(CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT);
    if (is_supported_resolution(thumbnailSizeValues, thumbnailSizeCount,
                &newThumbnailSize)) {
        thumbnailSize = newThumbnailSize;
    }

    newThumbnailQuality = p->getInt(CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY);
    if (newThumbnailQuality >= 1 && newThumbnailQuality <= 100) {
        thumbnailQuality = newThumbnailQuality;
    }

    /* Video */
    resolution_t newVideoSize;
    int newVideoFormat;
    bool newVideoStabilization;

    p->getVideoSize(&newVideoSize.width, &newVideoSize.height);
    if (is_supported_resolution(videoSizeValues, videoSizeCount,
                &newVideoSize)) {
        videoSize = newVideoSize;
    }

    newVideoFormat = string_to_v4l2_format(
            p->get(CameraParameters::KEY_VIDEO_FRAME_FORMAT));
    if (newVideoFormat & videoFormatValues) {
        videoFormat = newVideoFormat;
    }

    newVideoStabilization = get_bool(p, CameraParameters::KEY_VIDEO_STABILIZATION);
    videoStabilization = newVideoStabilization;

    /* Focus */
    int newFocusMode;

    paramStr = p->get(CameraParameters::KEY_FOCUS_MODE);
    newFocusMode = string_to_focus_mode(paramStr);
    if (newFocusMode & focusModeValues) {
        focusMode = newFocusMode;
    }

    /* TODO: handle focus areas */

    /* Zoom */
    int newZoom;

    newZoom = p->getInt(CameraParameters::KEY_ZOOM);
    if (newZoom >= 0 && newZoom <= zoomMax) {
        zoom = newZoom;
    }

    /* Flash */
    int newFlashMode;

    paramStr = p->get(CameraParameters::KEY_FLASH_MODE);
    newFlashMode = string_to_flash_mode(paramStr);
    if (newFlashMode & flashModeValues) {
        flashMode = newFlashMode;
    }

    /* Exposure compensation */
    int newExposureCompensation;

    newExposureCompensation = p->getInt(CameraParameters::KEY_EXPOSURE_COMPENSATION);
    if (newExposureCompensation >= exposureCompensationMin &&
            newExposureCompensation <= exposureCompensationMax) {
        exposureCompensation = newExposureCompensation;
    }

    /* Antibanding */
    int newAntibanding;

    paramStr = p->get(CameraParameters::KEY_ANTIBANDING);
    newAntibanding = string_to_antibanding(paramStr);
    if (newAntibanding & antiBandingValues) {
        antiBanding = newAntibanding;
    }

    /* White balance */
    int newWhiteBalance;

    paramStr = p->get(CameraParameters::KEY_WHITE_BALANCE);
    newWhiteBalance = string_to_white_balance(paramStr);
    if (newWhiteBalance & whiteBalanceValues) {
        whiteBalance = newWhiteBalance;
    }

    /* Scene */
    int newSceneMode;

    paramStr = p->get(CameraParameters::KEY_SCENE_MODE);
    newSceneMode = string_to_scene_mode(paramStr);
    if (newSceneMode & sceneModeValues) {
        sceneMode = newSceneMode;
    }

    /* Effect */
    int newEffect;

    paramStr = p->get(CameraParameters::KEY_EFFECT);
    newEffect = string_to_effect(paramStr);
    if (newEffect & effectValues) {
        effect = newEffect;
    }

    /* Angles */
    int newRotation;

    newRotation = p->getInt(CameraParameters::KEY_ROTATION);
    if (newRotation >= 0) {
        rotation = newRotation;
    }

    /* Misc */
    int newIso;

    paramStr = p->get("iso");
    newIso = string_to_iso(paramStr);
    /* TODO: check camera supported modes? */
    iso = newIso;

    /* Color */
    int newContrast;

    paramStr = p->get("contrast");
    newContrast = string_to_contrast(paramStr);
    contrast = newContrast;

    int newBrightness = p->getInt("brightness");
    if (newBrightness >= brightnessMin && newBrightness <= brightnessMax) {
        brightness = newBrightness;
    }

    int newHue = p->getInt("hue");
    if (newHue >= hueMin && newHue <= hueMax) {
        hue = newHue;
    }

    int newSaturation = p->getInt("saturation");
    if (newSaturation >= saturationMin && newSaturation <= saturationMax) {
        saturation = newSaturation;
    }

    int newSharpness = p->getInt("saturation");
    if (newSharpness >= sharpnessMin && newSharpness <= sharpnessMax) {
        sharpness = newSharpness;
    }
}

ExynosCameraInfoIMX135::ExynosCameraInfoIMX135()
{
    ALOGD("DEBUG(%s): Initializing", __FUNCTION__);

    cameraInfo.facing = CAMERA_FACING_BACK;
    cameraInfo.orientation = 0;

    previewSizeValues = imx135_previewTable;
    previewSizeCount = ARRAY_SIZE(imx135_previewTable);
    previewSize.width = 1920;
    previewSize.height = 1080;

    previewFormatValues = V4L2_PIX_FMT_NV21;
    previewFormat = V4L2_PIX_FMT_NV21;

    previewFpsCount = 24;
    previewFpsValues = new int[previewFpsCount];
    for (int i = 0; i < previewFpsCount; i++) {
        previewFpsValues[i] = 7 + i;
    }
    previewFps = 30;

    previewFpsRangeCount = 2;
    previewFpsRangeValues = new fps_range_t[previewFpsRangeCount];
    previewFpsRangeValues[0].min = 15000;
    previewFpsRangeValues[0].max = 30000;
    previewFpsRangeValues[1].min = 30000;
    previewFpsRangeValues[1].max = 30000;
    previewFpsRange.min = 15000;
    previewFpsRange.max = 30000;

    pictureSizeValues = imx135_pictureTable;
    pictureSizeCount = ARRAY_SIZE(imx135_pictureTable);
    pictureSize.width = 4128;
    pictureSize.height = 2322;
    pictureFormatValues = V4L2_PIX_FMT_YUYV;
    pictureFormat = V4L2_PIX_FMT_YUYV;
    pictureQuality = 100;

    thumbnailSizeValues = imx135_thumbnailTable;
    thumbnailSizeCount = ARRAY_SIZE(imx135_thumbnailTable);
    thumbnailSize.width = 512;
    thumbnailSize.height = 384;
    thumbnailQuality = 100;

    videoSizeValues = imx135_videoTable;
    videoSizeCount = ARRAY_SIZE(imx135_videoTable);
    videoSize.width = 1920;
    videoSize.height = 1080;
    videoPreferredPreviewSize.width = 1920;
    videoPreferredPreviewSize.height = 1080;
    videoFormatValues = V4L2_PIX_FMT_NV12M;
    videoFormat = V4L2_PIX_FMT_NV12M;
    videoStabilizationSupported = true;
    videoStabilization = false;

    focusModeValues =
          FOCUS_MODE_AUTO
        //| FOCUS_MODE_INFINITY
        | FOCUS_MODE_MACRO
        //| FOCUS_MODE_FIXED
        //| FOCUS_MODE_EDOF
        | FOCUS_MODE_CONTINUOUS_VIDEO
        | FOCUS_MODE_CONTINUOUS_PICTURE
        //| FOCUS_MODE_TOUCH
        ;
    focusMode = FOCUS_MODE_AUTO;
    focusDistances =
          FOCUS_DISTANCE_10
        | FOCUS_DISTANCE_120
        | FOCUS_DISTANCE_INF
        ;
    focusMaxAreas = 1;

    zoomSupported = true;
    zoomSmoothSupported = false;
    zoomRatioCount = ARRAY_SIZE(imx135_zoomRatioTable);
    zoomRatioValues = imx135_zoomRatioTable;
    zoomMax = 30;
    zoom = 0;

    flashModeValues =
          FLASH_MODE_OFF
        | FLASH_MODE_AUTO
        | FLASH_MODE_ON
        //| FLASH_MODE_RED_EYE
        | FLASH_MODE_TORCH;
    flashMode = FLASH_MODE_OFF;

    exposureCompensationMin = -4;
    exposureCompensationMax = 4;
    exposureCompensation = 0;
    exposureCompensationStep = 0.5f;

    antiBandingValues =
          ANTIBANDING_OFF
        | ANTIBANDING_50HZ
        | ANTIBANDING_60HZ
        | ANTIBANDING_OFF;
    antiBanding = ANTIBANDING_OFF;

    whiteBalanceValues =
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

    sceneModeValues =
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

    effectValues =
          EFFECT_NONE
        | EFFECT_MONO
        | EFFECT_NEGATIVE
        | EFFECT_SOLARIZE
        | EFFECT_SEPIA
        | EFFECT_POSTERIZE
        | EFFECT_WHITEBOARD
        | EFFECT_BLACKBOARD
        | EFFECT_AQUA
        ;
    effect = EFFECT_NONE;

    faceDetectionHwMax = 16;
    faceDetectionSwMax = 0;

    rotation = 0;

    meteringMaxAreas = 32;

    focalLength = 4.20f;

    viewAngleHorizontal = 51.2f;
    viewAngleVertical = 39.4f;

    iso = ISO_AUTO;

    contrast = CONTRAST_AUTO;

    brightnessMax = 2;
    brightnessMin = -2;
    brightness = 0;

    hueMax = 2;
    hueMin = -2;
    hue = 0;

    saturationMax = 2;
    saturationMin = -2;
    saturation = 0;

    sharpnessMax = 2;
    sharpnessMin = -2;
    sharpness = 0;

    vtmode = 0;
    wdr = 0;
}

ExynosCameraInfoS5K6B2::ExynosCameraInfoS5K6B2()
{
    ALOGD("DEBUG(%s): Initializing", __FUNCTION__);
}

}; // namespace android

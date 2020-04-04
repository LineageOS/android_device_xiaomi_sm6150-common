/*
 * Copyright (C) 2019-2020 The LineageOS Project
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

#define LOG_TAG "SunlightEnhancementService"

#include "SunlightEnhancement.h"

#include <android-base/logging.h>
#include <android-base/properties.h>
#include <android-base/strings.h>
#include <utils/Errors.h>

#include <fstream>

namespace {

/* clang-format off */
#define PPCAT_NX(A, B) A/B
#define PPCAT(A, B) PPCAT_NX(A, B)
#define STRINGIFY_INNER(x) #x
#define STRINGIFY(x) STRINGIFY_INNER(x)

#define DRM(x) PPCAT(/sys/class/drm/, x)
#define DSI(x) STRINGIFY(PPCAT(DRM(card0-DSI-1), x))
/* clang-format on */

}  // anonymous namespace

namespace vendor {
namespace lineage {
namespace livedisplay {
namespace V2_0 {
namespace implementation {

bool SunlightEnhancement::isSupported() {
    return true;
}

Return<bool> SunlightEnhancement::isEnabled() {
    std::ifstream hbm_status_file(DSI(hbm_status));
    int result = -1;
    hbm_status_file >> result;
    return !hbm_status_file.fail() && result > 0;
}

Return<bool> SunlightEnhancement::setEnabled(bool enabled) {
    xiaomiDisplayFeatureService = IDisplayFeature::getService();
    if (enabled) {
        xiaomiDisplayFeatureService->setFeature(0, 0, 2, 255);
        xiaomiDisplayFeatureService->setFeature(0, 11, 1, 3);
    } else {
        xiaomiDisplayFeatureService->setFeature(0, 11, 0, 3);
    }
    return false;
}

}  // namespace implementation
}  // namespace V2_0
}  // namespace livedisplay
}  // namespace lineage
}  // namespace vendor

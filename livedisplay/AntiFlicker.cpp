/*
 * Copyright (C) 2020 The LineageOS Project
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

#define LOG_TAG "AntiFlickerService"

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/strings.h>

#include "AntiFlicker.h"

namespace aidl {
namespace vendor {
namespace lineage {
namespace livedisplay {

static constexpr const char* kAntiFlickerStatusPath =
        "/sys/devices/platform/soc/soc:qcom,dsi-display/dc_enable";

ndk::ScopedAStatus AntiFlicker::getEnabled(bool* aidl_return) {
    std::string buf;
    if (!android::base::ReadFileToString(kAntiFlickerStatusPath, &buf)) {
        LOG(ERROR) << "Failed to read " << kAntiFlickerStatusPath;
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }
    *aidl_return = std::stoi(android::base::Trim(buf)) == 1;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus AntiFlicker::setEnabled(bool enabled) {
    if (!android::base::WriteStringToFile((enabled ? "1" : "0"), kAntiFlickerStatusPath)) {
        LOG(ERROR) << "Failed to write " << kAntiFlickerStatusPath;
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }
    return ndk::ScopedAStatus::ok();
}

}  // namespace livedisplay
}  // namespace lineage
}  // namespace vendor
}  // namespace aidl

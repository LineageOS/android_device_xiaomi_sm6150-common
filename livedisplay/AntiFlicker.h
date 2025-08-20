/*
 * SPDX-FileCopyrightText: 2019-2025 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <aidl/vendor/lineage/livedisplay/BnAntiFlicker.h>

namespace aidl {
namespace vendor {
namespace lineage {
namespace livedisplay {

class AntiFlicker : public BnAntiFlicker {
  public:
    // Methods from ::aidl::vendor::lineage::livedisplay::BnAntiFlicker follow.
    ndk::ScopedAStatus getEnabled(bool* aidl_return) override;
    ndk::ScopedAStatus setEnabled(bool enabled) override;
};

}  // namespace livedisplay
}  // namespace lineage
}  // namespace vendor
}  // namespace aidl

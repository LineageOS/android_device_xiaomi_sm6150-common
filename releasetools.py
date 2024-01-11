#
# Copyright (C) 2009 The Android Open Source Project
# Copyright (C) 2019 The Mokee Open Source Project
# Copyright (C) 2020-2021 The LineageOS Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import common
import os
import re

def FullOTA_InstallBegin(info):
  # Install super_dummy only if device is using retrofitted dynamic partitions
  if os.getenv("TARGET_PRODUCT").split("_")[1] in ("davinci", "tucana", "violet"):
    AddImage(info, "RADIO", "super_dummy.img", "/tmp/super_dummy.img")
    info.script.AppendExtra('package_extract_file("install/bin/flash_super_dummy.sh", "/tmp/flash_super_dummy.sh");')
    info.script.AppendExtra('set_metadata("/tmp/flash_super_dummy.sh", "uid", 0, "gid", 0, "mode", 0755);')
    info.script.AppendExtra('run_program("/tmp/flash_super_dummy.sh");')
  
  return

def FullOTA_Assertions(info):
  AddBasebandAssertion(info, info.input_zip)
  return

def IncrementalOTA_Assertions(info):
  AddBasebandAssertion(info, info.input_zip)
  return

def FullOTA_InstallEnd(info):
  OTA_InstallEnd(info)
  return

def IncrementalOTA_InstallEnd(info):
  OTA_InstallEnd(info)
  return

def AddImage(info, directory, basename, dest):
  name = basename
  path = directory + "/" + name
  if path not in info.input_zip.namelist():
    return

  data = info.input_zip.read(path)
  common.ZipWriteStr(info.output_zip, name, data)
  info.script.Print("Patching {} image unconditionally...".format(dest.split('/')[-1]))
  info.script.AppendExtra('package_extract_file("%s", "%s");' % (name, dest))

def OTA_InstallEnd(info):
  AddImage(info, "IMAGES", "dtbo.img", "/dev/block/bootdevice/by-name/dtbo")
  AddImage(info, "IMAGES", "vbmeta.img", "/dev/block/bootdevice/by-name/vbmeta")
  AddImage(info, "IMAGES", "vbmeta_system.img", "/dev/block/bootdevice/by-name/vbmeta_system")
  return

def AddBasebandAssertion(info, input_zip):
  android_info = input_zip.open("OTA/android-info.txt")
  for line in android_info.readlines():
    m = re.search(r'require\s+version-baseband\s*-\s*(.+)', line.decode('utf-8'))
    if m:
      hwc, modem_version, firmware_version = re.split('[=,]', m.group(1).rstrip())
      if (len(hwc) and len(modem_version) and len(firmware_version)):
        cmd = 'assert(getprop("ro.boot.hwc") == "{0}" && (xiaomi.verify_baseband("{1}") == "1" || abort("ERROR: This package requires firmware from MIUI {2} or newer. Please upgrade firmware and retry!");) || true);'
        info.script.AppendExtra(cmd.format(hwc, modem_version, firmware_version))
  return

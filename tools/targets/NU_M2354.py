#!/usr/bin/python
# Copyright (c) 2017-2021 Arm Limited
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from os.path import abspath, basename, dirname, splitext, isdir
from os.path import join as path_join
import re
import subprocess
import shutil

SCRIPT_DIR = dirname(abspath(__file__))
MBED_OS_ROOT = abspath(path_join(SCRIPT_DIR, os.pardir, os.pardir))

def m2354_tfm_bin(t_self, non_secure_bin, secure_bin):

    assert os.path.isfile(secure_bin)
    assert os.path.isfile(non_secure_bin)

    secure_bin = abspath(secure_bin)
    non_secure_bin = abspath(non_secure_bin)

    SECURE_ROOT = abspath(dirname(secure_bin))

    build_dir = dirname(non_secure_bin)
    tempdir = path_join(build_dir, 'temp')
    if not isdir(tempdir):
        os.makedirs(tempdir)

    flash_layout = path_join(SECURE_ROOT, 'partition', 'flash_layout.h')

    bl2_bin = path_join(SECURE_ROOT, 'bl2.bin')
    image_macros_s_ns = path_join(SECURE_ROOT, 'partition', 'signing_layout_preprocessed.h')
    ns_bin_name, ns_bin_ext = splitext(basename(non_secure_bin))
    concatenated_bin = abspath(path_join(tempdir, 'tfm_' + ns_bin_name + ns_bin_ext))
    signed_bin = abspath(path_join(tempdir, 'tfm_' + ns_bin_name + '_signed' + ns_bin_ext))

    assert os.path.isfile(image_macros_s_ns)

    signing_key = path_join(SCRIPT_DIR, 'nuvoton_m2354-root-rsa-3072.pem')
    assert os.path.isfile(signing_key)

    # Find Python 3 command name across platforms
    python3_cmd = "python3" if shutil.which("python3") is not None else "python"

    #1. Concatenate secure TFM and non-secure mbed binaries
    cmd = [
        python3_cmd,
        path_join(MBED_OS_ROOT, "tools", "psa","tfm", "bin_utils","assemble.py"),
        "--layout",
        image_macros_s_ns,
        "-s",
        secure_bin,
        "-n",
        non_secure_bin,
        "-o",
        concatenated_bin,
    ]

    retcode = run_cmd(cmd, MBED_OS_ROOT)
    if retcode:
        raise Exception("Unable to concatenate " + "TF-M Secure/Mbed Non-secure" +
                            " binaries, Error code: " + str(retcode))
        return

    #2. Run wrapper to sign the concatenated binary
    cmd = [
        python3_cmd,
        path_join(MBED_OS_ROOT, "tools", "psa","tfm", "bin_utils","wrapper.py"),
        "-v",
        '1.2.0',
        "-k",
        signing_key,
        "--layout",
        image_macros_s_ns,
        "--public-key-format",
        'full',
        "--align",
        '1',
        "--pad",
        "--pad-header",
        "-H",
        '0x400',
        "--overwrite-only",
        "-s",
        'auto',
        "-d",
        '(0,0.0.0+0)',
        concatenated_bin,
        signed_bin,
    ]

    retcode = run_cmd(cmd, MBED_OS_ROOT)
    if retcode:
        raise Exception("Unable to sign " + "concatenated" +
                            " binary, Error code: " + str(retcode))
        return

    #3. Concatenate mcuboot and signed binary and overwrite mbed built binary file
    flash_area_0_offset = find_flash_area_0_offset(flash_layout)
    with open(bl2_bin, "rb") as mcuboot_fh, open(signed_bin, "rb") as signed_fh:
        with open(non_secure_bin, "w+b") as out_fh:
            out_fh.write(mcuboot_fh.read())
            out_fh.seek(flash_area_0_offset)
            out_fh.write(signed_fh.read())

def find_flash_area_0_offset(configFile):
    # Compiled regular expressions 
    flash_area_bl2_offset_re = re.compile(r"^#define\s+FLASH_AREA_BL2_OFFSET\s+\({0,1}(0x[0-9a-fA-F]+)\){0,1}")
    flash_area_bl2_size_re = re.compile(r"^#define\s+FLASH_AREA_BL2_SIZE\s+\({0,1}(0x[0-9a-fA-F]+)\){0,1}")
    rsvd_stor_size_re = re.compile(r"^#define\s+FLASH_AREA_0_OFFSET\s+\(FLASH_AREA_BL2_OFFSET\s+\+\s+FLASH_AREA_BL2_SIZE\s+\+\s+\({0,1}(0x[0-9a-fA-F]+)\){0,1}\)")

    # Match values
    flash_area_bl2_offset = None
    flash_area_bl2_size = None
    rsvd_stor_size = None
    flash_area_0_offset = None

    with open(configFile, 'r') as configFile_:
        for line in configFile_:
            # Seek "#define FLASH_AREA_BL2_OFFSET..."
            if flash_area_bl2_offset is None:
                m = flash_area_bl2_offset_re.match(line)
                if m is not None:
                    flash_area_bl2_offset = int(m.group(1), 0)
                    continue

            # Seek "#define FLASH_AREA_BL2_SIZE..."
            if flash_area_bl2_size is None:
                m = flash_area_bl2_size_re.match(line)
                if m is not None:
                    flash_area_bl2_size = int(m.group(1), 0)
                    continue

            # Seek "#define FLASH_AREA_0_OFFSET..."
            if rsvd_stor_size is None:
                m = rsvd_stor_size_re.match(line)
                if m is not None:
                    rsvd_stor_size = int(m.group(1), 0)
                    continue

            # FLASH_AREA_0_OFFSET = FLASH_AREA_BL2_OFFSET + FLASH_AREA_BL2_SIZE + Reserved storage area size
            if flash_area_bl2_offset is not None and \
                flash_area_bl2_size is not None and \
                rsvd_stor_size is not None:
                flash_area_0_offset = flash_area_bl2_offset + flash_area_bl2_size + rsvd_stor_size
                break

    return flash_area_0_offset

def run_cmd(cmd, directory):

    # Redirect stdout/stderr to pipe, text mode
    POPEN_INSTANCE = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        cwd=directory,
        universal_newlines=True
    )

    # Command line
    print("COMMAND: {}".format(POPEN_INSTANCE.args))

    stdout_data, stderr_data = POPEN_INSTANCE.communicate()

    # stdout/stderr messages
    if (stdout_data):
        print(stdout_data)
    if (stderr_data):
        print(stderr_data)

    # Return code
    return POPEN_INSTANCE.returncode

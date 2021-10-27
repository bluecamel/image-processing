#!/usr/bin/env bash

set -eoux

HERE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd "${HERE}"

# Download source.
curl -L "http://rpg.ifi.uzh.ch/software/ocamcalib/Scaramuzza_OCamCalib_v3.0_win.zip" \
    --output Scaramuzza_OCamCalib_v3.0_win.zip
curl -L "http://rpg.ifi.uzh.ch/software/ocamcalib/autoCornerFinder_linux.zip" \
    --output autoCornerFinder_linux.zip

# Unzip source.
unzip Scaramuzza_OCamCalib_v3.0_win.zip
unzip autoCornerFinder_linux.zip

# Patch for modern OpenCV (tested with 4.2.0 and 4.5.3).
patch -R autoCornerFinder_linux/cvcalibinit3.cpp patch/autoCornerFinder_linux/cvcalibinit3.cpp.patch
patch -R autoCornerFinder_linux/cvcalibinit3.h patch/autoCornerFinder_linux/cvcalibinit3.h.patch
patch -R autoCornerFinder_linux/main.cpp patch/autoCornerFinder_linux/main.cpp.patch
patch -R autoCornerFinder_linux/Makefile patch/autoCornerFinder_linux/Makefile.patch

# Compile FindCorners.exe and move into place.
cd  "${HERE}/autoCornerFinder_linux"
make
cp FindCorners.exe "${HERE}/Scaramuzza_OCamCalib_v3.0_win/autoCornerFinder/"
chmod u+x "${HERE}/Scaramuzza_OCamCalib_v3.0_win/autoCornerFinder/FindCorners.exe"

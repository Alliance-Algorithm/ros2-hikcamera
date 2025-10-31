#!/bin/bash

SRC="$1"
DST="$2"

if [ ! -d "$SRC" ]; then
    echo "❌ Source path not found: $SRC"
    exit 1
fi

if [ ! -d "$DST" ]; then
    echo "❌ Target path not found: $DST"
    exit 1
fi

mkdir -p "$DST/lib"

FILES=(
    "MvFGProducerCML.cti"
    "MvFGProducerCXP.cti"
    "MvFGProducerGEV.cti"
    "MvFGProducerXoF.cti"
    "MvProducerGEV.cti"
    "MvProducerU3V.cti"
    "libCLAllSerial_gcc485_v3_0.so"
    "libCLProtocol_gcc485_v3_0.so"
    "libCLSerCOM.so"
    "libCLSerHvc.so"
    "libFormatConversion.so"
    "libGCBase_gcc485_v3_0.so"
    "libGenCP_gcc485_v3_0.so"
    "libLog_gcc485_v3_0.so"
    "liblog4cpp_gcc485_v3_0.so"
    "libMediaProcess.so"
    "libMVFGControl.so"
    "libMVGigEVisionSDK.so"
    "libMVGigEVisionSDK.so."*
    "libMVRender.so"
    "libMvCamLVision.so"
    "libMvCamLVision.so."*
    "libMvCameraControl.so"
    "libMvCameraControl.so."*
    "libMvCameraControlWrapper.so"
    "libMvCameraControlWrapper.so."*
    "libMvUsb3vTL.so"
    "libMvUsb3vTL.so."*
    "libavutil.so"
    "libswscale.so"
)

SEARCH_PATHS=(
    "$SRC/lib/64"
    "$SRC/lib/64/ThirdParty"
    "$SRC/lib/CLProtocol/Linux64_x64"
)

success_count=0
fail_count=0

for pattern in "${FILES[@]}"; do
    found=false
    for dir in "${SEARCH_PATHS[@]}"; do
        for src in "$dir"/$pattern; do
            if [ -e "$src" ]; then
                echo "📦 copying $(basename "$src") from $dir"
                cp -a "$src" "$DST/lib/"
                success_count=$((success_count + 1))
                found=true
            fi
        done
        if $found; then break; fi
    done
    if ! $found; then
        echo "⚠️  not found: $pattern"
        fail_count=$((fail_count + 1))
    fi
done

echo "✅ All files processed."
echo "✅ Success: $success_count"
echo "⚠️  Failed:  $fail_count"

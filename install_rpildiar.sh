#!/bin/bash

TEMP_SOURCE=/tmp/rplidar
TEMP_DIR=${TEMP_SOURCE}/rplidar_sdk-2025
TEMP_SDK_DIR=${TEMP_SOURCE}/rplidar_sdk-2025/sdk
THIRD_PARTY=third_party/rplidar

rm -rf ${TEMP_SOURCE}
mkdir ${TEMP_SOURCE}
cd ${TEMP_SOURCE}
curl -L https://github.com/bashtavenko/rplidar_sdk/archive/refs/tags/v2025.tar.gz | tar xz
make -C ${TEMP_SDK_DIR}

# May need to comment out above in order to copy
rm -rf ${THIRD_PARTY}
mkdir -p ${THIRD_PARTY}
cp -r ${TEMP_SDK_DIR}/include ${THIRD_PARTY}
cp ${TEMP_DIR}/output/Linux/Release/libsl_lidar_sdk.a ${THIRD_PARTY}
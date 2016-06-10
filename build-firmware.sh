#!/bin/bash
#
# This script builds the FPGA IF Card firmware. Run it in Nios II console!
#
# Steps:
# - Build Quartus project
# - Build Nios II project
# - Update SOF Bootloader
# - Update the firmware update image binary
#
# ./build-firmware.sh

BASE_DIR=${PWD}
OPLK_DIR=oplk

QUARTUS_PROJECT_PATH=hardware/boards/br-antaresif/mn-single-pcie-drv/quartus

NIOS2_PROJECT_PATH=drivers/altera-nios2/drv_daemon/build
NIOS2_PROJECT_CREATEFILE=create-this-app
NIOS2_PROJECT_MAKEFILE=Makefile

CREATE_UPDATE_IMAGE_PATH=tools/altera-nios2
CREATE_UPDATE_IMAGE_FILE=create-update-image.sh
UPDATE_FIRMWARE_IMAGE_FILE=image_hdr.bin

APP_FIRMWARE_TOOL_PATH=apps/firmware_update/src
APP_FIRMWARE_TOOL_IMAGE_NAME=image.bin

# Obtain firmware information
FIRMWARE_INFO_FILE=firmware.info
FIRMWARE_VERSION=
FIRMWARE_FEATURE=

source $FIRMWARE_INFO_FILE

if [ -z "${FIRMWARE_VERSION}" ]; then
    echo "ERROR: No FIRMWARE_VERSION found in ${FIRMWARE_INFO_FILE}!"
    exit 1
fi

if [ -z "${FIRMWARE_FEATURE}" ]; then
    echo "ERROR: No FIRMWARE_FEATURE found in ${FIRMWARE_INFO_FILE}!"
    exit 1
fi

# - Build Quartus project
echo "INFO: Run make in ${QUARTUS_PROJECT_PATH}..."
echo

make -C ${QUARTUS_PROJECT_PATH}
if [ $? -ne 0 ]; then
    echo "ERROR: Building Quartus project failed!"
    exit 1
fi

# - Build Nios II project
echo "INFO: Run create-this-app and make in ${NIOS2_PROJECT_PATH}..."

## First change to the Nios II project path
pushd ${NIOS2_PROJECT_PATH} > /dev/null

## Now check if there is already a makefile available, in case we can skip creation!
if [ ! -f "${NIOS2_PROJECT_MAKEFILE}" ]; then
    chmod +x ${NIOS2_PROJECT_CREATEFILE}
    ./${NIOS2_PROJECT_CREATEFILE}
    if [ $? -ne 0 ]; then
        echo "ERROR: Creating Nios II Makefile failed!"
        popd > /dev/null
        exit 1
    fi
fi

## Now run make and cross your fingers!
make
if [ $? -ne 0 ]; then
    echo "ERROR: Nios II make failed!"
    exit 1
fi

popd > /dev/null

# - Update SOF Bootloader
echo "INFO: Update epcs bootloader in sof..."

make -C ${NIOS2_PROJECT_PATH} update-epcsboot
if [ $? -ne 0 ]; then
    echo "ERROR: Updating bootloader in sof failed!"
    exit 1
fi

# - Update the firmware update image binary
echo "INFO: Update firmware update image binary..."

# First copy sof and elf to tools dir
cp ${QUARTUS_PROJECT_PATH}/*.sof ${CREATE_UPDATE_IMAGE_PATH}
if [ $? -ne 0 ]; then
    echo "ERROR: Couldn't get .sof from ${QUARTUS_PROJECT_PATH}!"
    exit 1
fi

cp ${NIOS2_PROJECT_PATH}/*.elf ${CREATE_UPDATE_IMAGE_PATH}
if [ $? -ne 0 ]; then
    echo "ERROR: Couldn't get .elf from ${NIOS2_PROJECT_PATH}!"
    exit 1
fi

# Now change to the tools dir
pushd ${CREATE_UPDATE_IMAGE_PATH} > /dev/null

# And run the image creator script
chmod +x ${CREATE_UPDATE_IMAGE_FILE}
./${CREATE_UPDATE_IMAGE_FILE} *.sof *.elf ${FIRMWARE_VERSION} ${FIRMWARE_FEATURE}
if [ $? -ne 0 ]; then
    echo "ERROR: Couldn't create firmware image successful!"
    popd > /dev/null
    exit 1
fi

popd > /dev/null

# Finaly copy the image from the tools dir to apps
CP_SRC_PATH=${CREATE_UPDATE_IMAGE_PATH}/${UPDATE_FIRMWARE_IMAGE_FILE}
CP_DST_PATH=${APP_FIRMWARE_TOOL_PATH}/${APP_FIRMWARE_TOOL_IMAGE_NAME}

cp $CP_SRC_PATH $CP_DST_PATH
if [ $? -ne 0 ]; then
    echo "ERROR: Couldn't copy update image to app dir!"
    exit 1
fi

# Cleanup before leaving
rm ${CREATE_UPDATE_IMAGE_PATH}/*.sof -f
rm ${CREATE_UPDATE_IMAGE_PATH}/*.elf -f
rm ${CREATE_UPDATE_IMAGE_PATH}/*.bin -f

exit 0

#!/bin/bash
#
# This script builds the FPGA design with the correct bootloader.
#
# ./build_fpga.sh

APC_ROOT_DIR=../../../../..
QUARTUS_PRJ_DIR=$PWD/..
BOOTLOADER_DIR=${APC_ROOT_DIR}/oplk/contrib/bootloader/altera-nios2/epcsboot/build

# Start with creating the Qsys design first
make qsys
if [ $? -ne 0 ]; then
    exit 1
fi

echo

# Change to bootloader dir, create it and exchange hex in Quartus project dir
pushd $BOOTLOADER_DIR

./build_bootloader.sh ${QUARTUS_PRJ_DIR}
if [ $? -ne 0 ]; then
    popd
    exit 1
fi

popd

echo

# Complete compilation of Quartus design
make all
if [ $? -ne 0 ]; then
    exit 1
fi

exit 0

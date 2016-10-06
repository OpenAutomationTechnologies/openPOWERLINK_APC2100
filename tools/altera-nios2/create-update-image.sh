#!/bin/bash
# $ create-update-image.sh [SOF_FILE] [ELF_FILE] [OPLK_VERSION] [OPLK_FEATURE]

SOF_FILE=$1
ELF_FILE=$2
OPLK_VERSION=$3
OPLK_FEATURE=$4

printHelp()
{
    echo "Usage:"
    echo " ./create-update-image.sh [SOF_FILE] [ELF_FILE] [OPLK_VERSION] [OPLK_FEATURE]"
    echo
    echo "Parameters:"
    echo "[SOF_FILE] ...... Path to .sof file"
    echo "[ELF_FILE] ...... Path to .elf file"
    echo "[OPLK_VERSION] .. OPLK version in 32 bit hex"
    echo "[OPLK_FEATURE] .. OPLK version in 32 bit hex"
}

# Check number of inputs
if [ $# -lt 4 ]; then
    printHelp
    exit 1
fi

HW_FILE=hw
SW_FILE=sw
IMG_FILE=image

MAKE_HEADER_PL=make_header.pl

SIGNATUR=0x46575550
VERSION=0x00000001

if [ ! -f "${MAKE_HEADER_PL}" ]; then
    echo "ERROR: Could not find ${MAKE_HEADER_PL}!"
    echo "       ${MAKE_HEADER_PL} must be stored in same path!"
    exit 1
fi

# Create flash files...
echo
echo "INFO: Create flash file for bitstream ${SOF_FILE} ..."
CMD="sof2flash --epcs --input=${SOF_FILE} --output=${HW_FILE}.flash --compress"
${CMD} || {
    echo
    echo "ERROR: SOF2FLASH failed!"
    exit 1
}

echo
echo "INFO: Create flash file for elf ${ELF_FILE} ..."
CMD="elf2flash --epcs --input=${ELF_FILE} --outfile=${SW_FILE}.flash \
--after=${HW_FILE}.flash \
"
${CMD} || {
    echo
    echo "ERROR: ELF2FLASH failed!"
    exit 1
}

# Concatenate hard- and software srecs
echo
echo "INFO: Concatenate bitstream and elf to a single flash file..."
cat ${HW_FILE}.flash ${SW_FILE}.flash > ${IMG_FILE}.flash

# Create binary out of flash file
echo
echo "INFO: Create binary out of flash file..."
CMD="nios2-elf-objcopy -I srec -O binary ${IMG_FILE}.flash ${IMG_FILE}.bin"
${CMD} || {
    echo
    echo "ERROR: OBJCOPY hardware srec to binary failed!"
    exit 1
}

# Remove srecs
rm -rf ${HW_FILE}.flash ${SW_FILE}.flash ${IMG_FILE}.flash

# Add header to binary file
CMD="\
perl ./${MAKE_HEADER_PL} \
${IMG_FILE}.bin ${IMG_FILE}_hdr.bin \
${SIGNATUR} ${VERSION} ${OPLK_VERSION} ${OPLK_FEATURE} \
"
${CMD} || {
    echo
    echo "ERROR: Make header perl script failed!"
    exit 1
}

exit 0

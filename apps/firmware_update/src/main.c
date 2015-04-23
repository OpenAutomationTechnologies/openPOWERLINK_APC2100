/**
********************************************************************************
\file   main.c

\brief  Main file of firmware update tool for APC/PPC2100

This file contains the main file of the openPOWERLINK firmware update tool.

\ingroup module_fwupdate_tool
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/
#define _CRT_NONSTDC_NO_WARNINGS    // for MSVC 2005 or higher
//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdio.h>
#include <limits.h>
#include <string.h>
#include <sys/stat.h>
#include <assert.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <io.h>
#include <sys/types.h>
#include <sys/utime.h>
#include <sys/timeb.h>
#include <time.h>
#include <direct.h>


#include <oplk/oplk.h>
#include <oplk/debugstr.h>
#include <ctrlu.h>
#include <system/system.h>
#include <getopt/getopt.h>
#include <console/console.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define CYCLE_LEN         UINT_MAX
#define NODEID            0xF0                //=> MN
#define IP_ADDR           0xc0a86401          // 192.168.100.1
#define SUBNET_MASK       0xFFFFFF00          // 255.255.255.0
#define DEFAULT_GATEWAY   0xC0A864FE          // 192.168.100.C_ADR_RT1_DEF_NODE_ID
#define FW_HEADER_SIZE    32                  // In bytes

#define READ_FILE
//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
const BYTE aMacAddr_g[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifndef READ_FILE
const unsigned char     aTestBuffer[] =
{
    #include "image.txt"
};
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    char        fwImage[256];
    BOOL        fInvalidateUpdateImage;
} tOptions;

typedef struct
{
    char*   pFwImageBuff;
    INT     length;
} tFirmwareImage;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static int        getOptions(int argc_p, char** argv_p, tOptions* pOpts_p);
static tOplkError initPowerlink(UINT32 cycleLen_p, const BYTE* macAddr_p);
static void       shutdownPowerlink(void);
static tOplkError updateFirmwareImage(UINT8* pFwBuffer_p, INT length_p);
static tOplkError invalidateFirmwareImage(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  main function

This is the main function of the openPOWERLINK console MN demo application.

\param  argc                    Number of arguments
\param  argv                    Pointer to argument strings

\return Returns an exit code

\ingroup module_demo_mn_console
*/
//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    tOplkError                  ret = kErrorOk;
    tOptions                    opts;
    UINT32                      version;
    tFirmwareImage              firmwareImage;

    memset(&opts, 0, sizeof(tOptions));

    if (getOptions(argc, argv, &opts) == -1)
    {
        return -1;
    }

    if (system_init() != 0)
    {
        fprintf(stderr, "Error initializing system!");
        return 0;
    }

    version = PLK_DEFINED_STACK_VERSION;
    printf("----------------------------------------------------\n");
    printf("Firmware Update application for B&R APC/PPC2100\n");
    printf("for openPOWERLINK Stack: %x.%x.%x\n", PLK_STACK_VER(version), PLK_STACK_REF(version), PLK_STACK_REL(version));
    printf("----------------------------------------------------\n");

    if ((ret = initPowerlink(CYCLE_LEN, aMacAddr_g)) != kErrorOk)
    {
        printf("POWERLINK initialization failed error 0x%2X\n", ret);
        goto Exit;
    }

    if (opts.fInvalidateUpdateImage)
    {
        // Erase the existing update image and fallback to factory image
        ret = invalidateFirmwareImage();
        if (ret != kErrorOk)
            goto ExitFail;
        else
            goto Exit;
    }

#ifdef READ_FILE
    ret = readFirmwareFile(opts.fwImage, &firmwareImage);

    if (ret != kErrorOk)
    {
        printf("Unable to read firmware image error 0x%2X\n", ret);
        goto Exit;
    }
#else
    firmwareImage.pFwImageBuff = (unsigned char*) aTestBuffer;
    firmwareImage.length = (INT)sizeof(aTestBuffer);
#endif
    ret = updateFirmwareImage(firmwareImage.pFwImageBuff, firmwareImage.length);
    if (ret != kErrorOk)
        goto ExitFail;

Exit:
    shutdownPowerlink();
ExitFail:
    system_exit();

    if (ret == kErrorOk)
        printf("Please reboot the APC/PPC2100 to complete the firmware update\n");

    return 0;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Initialize the openPOWERLINK stack

The function initializes the openPOWERLINK stack.

\param  cycleLen_p              Length of POWERLINK cycle.
\param  macAddr_p               MAC address to use for POWERLINK interface.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initPowerlink(UINT32 cycleLen_p, const BYTE* macAddr_p)
{
    tOplkError                  ret = kErrorOk;
    static tOplkApiInitParam    initParam;
    static char                 devName[128];

    printf("Initializing openPOWERLINK stack...\n");

    memset(&initParam, 0, sizeof(initParam));
    initParam.sizeOfInitParam = sizeof(initParam);

    // pass selected device name to Edrv
    initParam.hwParam.pDevName = devName;
    initParam.nodeId = NODEID;
    initParam.ipAddress = (0xFFFFFF00 & IP_ADDR) | initParam.nodeId;

    /* write 00:00:00:00:00:00 to MAC address, so that the driver uses the real hardware address */
    memcpy(initParam.aMacAddress, macAddr_p, sizeof(initParam.aMacAddress));

    initParam.fAsyncOnly              = FALSE;
    initParam.featureFlags            = UINT_MAX;
    initParam.cycleLen                = cycleLen_p;       // required for error detection
    initParam.isochrTxMaxPayload      = 256;              // const
    initParam.isochrRxMaxPayload      = 256;              // const
    initParam.presMaxLatency          = 50000;            // const; only required for IdentRes
    initParam.preqActPayloadLimit     = 36;               // required for initialisation (+28 bytes)
    initParam.presActPayloadLimit     = 36;               // required for initialisation of Pres frame (+28 bytes)
    initParam.asndMaxLatency          = 150000;           // const; only required for IdentRes
    initParam.multiplCylceCnt         = 0;                // required for error detection
    initParam.asyncMtu                = 1500;             // required to set up max frame size
    initParam.prescaler               = 2;                // required for sync
    initParam.lossOfFrameTolerance    = 500000;
    initParam.asyncSlotTimeout        = 3000000;
    initParam.waitSocPreq             = 1000;
    initParam.deviceType              = UINT_MAX;         // NMT_DeviceType_U32
    initParam.vendorId                = UINT_MAX;         // NMT_IdentityObject_REC.VendorId_U32
    initParam.productCode             = UINT_MAX;         // NMT_IdentityObject_REC.ProductCode_U32
    initParam.revisionNumber          = UINT_MAX;         // NMT_IdentityObject_REC.RevisionNo_U32
    initParam.serialNumber            = UINT_MAX;         // NMT_IdentityObject_REC.SerialNo_U32

    initParam.subnetMask              = SUBNET_MASK;
    initParam.defaultGateway          = DEFAULT_GATEWAY;
    sprintf((char*)initParam.sHostname, "%02x-%08x", initParam.nodeId, initParam.vendorId);
    initParam.syncNodeId              = C_ADR_SYNC_ON_SOA;
    initParam.fSyncOnPrcNode          = FALSE;

    // set callback functions
    initParam.pfnCbEvent = NULL;
    initParam.pfnCbSync  = NULL;

    // initialize POWERLINK stack
    ret = ctrlu_init();
    if (ret != kErrorOk)
    {
        fprintf(stderr, "ctrlu_init() failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown the kernel stack

The function shuts down the kernel stack.
*/
//------------------------------------------------------------------------------
static void shutdownPowerlink(void)
{
    ctrlu_shutdownStack();
    ctrlu_exit();
}

//------------------------------------------------------------------------------
/**
\brief  Get command line parameters

The function parses the supplied command line parameters and stores the
options at pOpts_p.

\param  argc_p                  Argument count.
\param  argc_p                  Pointer to arguments.
\param  pOpts_p                 Pointer to store options

\return The function returns the parsing status.
\retval 0           Successfully parsed
\retval -1          Parsing error
*/
//------------------------------------------------------------------------------
static int getOptions(int argc_p, char** argv_p, tOptions* pOpts_p)
{
    int                         opt;

    /* setup default parameters */
    strncpy(pOpts_p->fwImage, "image.bin", 256);

    /* get command line parameters */
    while ((opt = getopt(argc_p, argv_p, "i:e")) != -1)
    {
        switch (opt)
        {
            case 'i':
                strncpy(pOpts_p->fwImage, optarg, 256);
                break;
            case 'e':
                pOpts_p->fInvalidateUpdateImage = TRUE;
                break;
            default: /* '?' */
                printf("Usage: %s [-i <UPDATE IMAGE> -e] \n"
                       "-i <UPDATE IMAGE>: Use the specified update image\n"
                       "-e : Invalidate the existing update image\n", argv_p[0]);
                return -1;
        }
    }
    return 0;
}

#ifdef READ_FILE
//------------------------------------------------------------------------------
/**
\brief  Read firmware image to update

The function reads the specied firware image file and store into the
intermediate buffer used to copy into the device flash.

\param  pFwBuffer_p        Pointer to the buffer containing update image.
\param  length_p           Length of the update image.

\return The function returns the tOplkError error code.

*/
//------------------------------------------------------------------------------
static tOplkError readFirmwareFile(char* pszFwFileName_p, tFirmwareImage* pFirmwareImage_p)
{
    int         fwFileHandle;
    int         readSize = 0;
    int         readlength;

    printf("Reading firmware update image %s\n", pszFwFileName_p);

    fwFileHandle = open(pszFwFileName_p, O_RDONLY | O_BINARY, 0666);

    if (fwFileHandle < 0)
    {   // error occurred
        errno = (UINT32) errno;
        printf("Unable to open file handle %x\n", errno);
        return kErrorNoResource;
    }

    pFirmwareImage_p->length = lseek(fwFileHandle, 0, SEEK_END);
    lseek(fwFileHandle, 0, SEEK_SET);

    pFirmwareImage_p->pFwImageBuff = malloc(pFirmwareImage_p->length);

    if (pFirmwareImage_p->pFwImageBuff == NULL)
    {
        return kErrorNoResource;
    }

    readlength = pFirmwareImage_p->length;

    do
    {
        readSize = read(fwFileHandle, pFirmwareImage_p->pFwImageBuff, readlength);

        if (readSize <= 0)
        {
            printf("Unable to read file\n");
            return kErrorNoResource;
        }

        readlength -= readSize;
        pFirmwareImage_p->pFwImageBuff = (unsigned char*) (pFirmwareImage_p->pFwImageBuff + readlength);

    } while (readlength > 0);


    close(fwFileHandle);
    return kErrorOk;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Write new firmware image

The function copies the new firmware image into the update image location in
flash.

\param  pFwBuffer_p        Pointer to the buffer containing update image.
\param  length_p           Length of the update image.

\return The function returns the tOplkError error code.

*/
//------------------------------------------------------------------------------
static tOplkError updateFirmwareImage(UINT8* pFwBuffer_p, INT length_p)
{
    tOplkError      ret = kErrorOk;
    tCtrlFileType   fileType = kCtrlFileTypeFirmwareUpdate;

    printf("Transfer firmware update image to kernel stack with size %d...\n",
                        (INT) length_p);
    ret = ctrlu_writeFileToKernel(fileType, length_p, pFwBuffer_p);
    if (ret != kErrorOk)
    {
        printf("ctrlu_writeFileToKernel() returned with 0x%X\n", ret);
        return ret;
    }

    printf("Updating next image flag to boot from new image on next boot...");
    ret = ctrlu_setNextImageFlag(fileType);

    if (ret != kErrorOk)
    {
        printf("Failed with error:0x%2X\n", ret);
        return ret;
    }

    printf("DONE!\n");

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Invalidate firmware image

The function invalidates the update image by erasing the firmware header and
instructs the firmware to reconfigure from factory image.

\return The function returns the tOplkError error code.

*/
//------------------------------------------------------------------------------
static tOplkError invalidateFirmwareImage(void)
{
    tOplkError      ret = kErrorOk;
    tCtrlFileType   fileType = kCtrlFileTypeFirmwareUpdate;
    UINT8*          pDummyFileBuff = malloc(FW_HEADER_SIZE);

    // Clear the dummy file buffer 
    memset(pDummyFileBuff, 0xFF, FW_HEADER_SIZE);

    printf("Erasing the update image....\n");

    ret = ctrlu_writeFileToKernel(fileType, FW_HEADER_SIZE, pDummyFileBuff);
    if (ret != kErrorOk)
    {
        printf("ctrlu_writeFileToKernel() returned with 0x%X\n", ret);
        return ret;
    }

    fileType = kCtrlFileTypeUnknown;

    printf("Updating next image flag to boot from factory image...");
    ret = ctrlu_setNextImageFlag(fileType);

    if (ret != kErrorOk)
    {
        printf("Failed with error:0x%2X\n", ret);
        return ret;
    }

    printf("DONE!\n");
    return ret;
}

/// \}

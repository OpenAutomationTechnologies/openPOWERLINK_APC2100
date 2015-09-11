/**
********************************************************************************
\file   daemon.c

\brief  POWERLINK FPGA Master daemon for PCP (kernel part)

This is the daemon for the PCP (kernel part) of the Altera Nios II POWERLINK
master demo application.

\ingroup module_daemon
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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


//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <system.h>
#include <sys/alt_cache.h>
#include <unistd.h>
#include <altera_avalon_pio_regs.h>

#include <oplk/oplk.h>
#include <oplk/debugstr.h>
#include <kernel/ctrlk.h>
#include <kernel/ctrlkcal.h>

#include <flash.h>
#include <firmware.h>
#include <prodtest.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define MAC_IS_ZERO(aMac)   ((aMac[0] == 0) && (aMac[1] == 0) && \
                             (aMac[2] == 0) && (aMac[3] == 0) && \
                             (aMac[4] == 0) && (aMac[5] == 0))

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    tFlashInfo          flashInfo;          ///< Flash info
    UINT32              writeOffset;        ///< Current flash write offset
    UINT32              writeEraseOffset;   ///< Current flash erase offset
    tFirmwareImageType  nextImage;          ///< Next firmware image to be configured
    BOOL                fStackInitialized;  ///< Stack is initialized
    size_t              fileChunkBufferSize; ///< Size of file chunk buffer
    UINT8*              pFileChunkBuffer;   ///< Buffer for file chunk transfer
} tDrvInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tDrvInstance drvInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError initPlk(void);
static void shtdPlk(void);
static void bgtPlk(void);
static BOOL ctrlCommandExecCb(tCtrlCmdType cmd_p, UINT16* pRet_p, UINT16* pStatus_p,
                              BOOL* pfExit_p);
static tOplkError writeFileChunk(void);
static tOplkError setNextReconfigFirmware(tFirmwareImageType imageType_p);
static tOplkError checkUpdateImage(void);
static tOplkError getMacAddress(UINT8* pMacAddr_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Main function

Calls the POWERLINK initialization and background task

\return 0

\ingroup module_daemon
*/
//------------------------------------------------------------------------------
int main(void)
{
    tOplkError ret;

    alt_icache_flush_all();
    alt_dcache_flush_all();

    PRINTF("CPU NIOS II /%s (%s)\n", ALT_CPU_CPU_IMPLEMENTATION, ALT_CPU_NAME);
    PRINTF("FREQ = %d MHZ\n", ALT_CPU_CPU_FREQ / 1000000U);
    PRINTF("DCACHE = %d BYTE\n", ALT_CPU_DCACHE_SIZE);
    PRINTF("ICACHE = %d BYTE\n", ALT_CPU_ICACHE_SIZE);

    while (1)
    {
        PRINTF("\n");

        memset((void*)&drvInstance_l, 0, sizeof(tDrvInstance));

        if (flash_init() != 0)
        {
            PRINTF("Flash initialize failed!\n");
            break;
        }

        if (firmware_init() != 0)
        {
            PRINTF("Firmware initialize failed!\n");
            break;
        }

        flash_getInfo(&drvInstance_l.flashInfo);

        switch (firmware_getCurrentImageType())
        {
            case kFirmwareImageFactory:
            {
                tFirmwareStatus firmwareStatus = firmware_getStatus();

                PRINTF("Firmware in factory image mode\n");
                PRINTF(" -> Firmware status = %d\n", firmwareStatus);

                if ((firmwareStatus == kFirmwareStatusPor) ||
                    (firmwareStatus == kFirmwareStatusReconfig))
                {
                    PRINTF(" -> Check for valid update image...\n");
                    if (setNextReconfigFirmware(kFirmwareImageUpdate) == kErrorOk)
                    {
                        PRINTF(" --> Valid image found, trigger reconfig!\n");
                        PRINTF("halt terminal\n%c", 4);
#ifndef NDEBUG
                        usleep(2000000U);
#endif
                        firmware_reconfig(kFirmwareImageUpdate);
                    }
                }
            }
                break;

            case kFirmwareImageUpdate:
                PRINTF("Firmware in update image mode\n");
                // We are in update image, nothing to do...
            default:
                break;
        }

        if (prodtest_init() != 0)
        {
            PRINTF("Production test initialize failed\n");
            break;
        }

        ret = initPlk();

        PRINTF("Initialization returned with \"%s\" (0x%X)\n",
               debugstr_getRetValStr(ret), ret);

        if (ret != kErrorOk)
            break;

        bgtPlk();

        PRINTF("Background loop stopped.\nShutdown Kernel Stack\n");

        shtdPlk();

        if (drvInstance_l.nextImage != kFirmwareImageUnknown)
        {
            usleep(2000000U); //jz: Wait here for some longer time!
            PRINTF("halt terminal\n%c", 4);
            firmware_reconfig(drvInstance_l.nextImage);
        }

        prodtest_exit();
        firmware_exit();
        flash_exit();

        usleep(1000000U);
    }

    PRINTF("halt terminal\n%c", 4);

    return 0;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief    openPOWERLINK stack initialization

This function initializes the communication stack and configures objects.

\return This function returns tOplkError error codes.
*/
//------------------------------------------------------------------------------
static tOplkError initPlk(void)
{
    tOplkError ret;


    ret = ctrlk_init(ctrlCommandExecCb);

    if (ret != kErrorOk)
    {
        PRINTF("Could not initialize control module\n");
        goto Exit;
    }

    drvInstance_l.fileChunkBufferSize = ctrlkcal_getMaxFileChunkSize();

    if (drvInstance_l.fileChunkBufferSize > 0)
    {
        drvInstance_l.pFileChunkBuffer = OPLK_MALLOC(drvInstance_l.fileChunkBufferSize);
        if (drvInstance_l.pFileChunkBuffer == NULL)
            ret = kErrorNoResource;
    }

Exit:
    return ret;
}


//------------------------------------------------------------------------------
/**
\brief    openPOWERLINK stack shutdown

This function shuts down the communication stack.
*/
//------------------------------------------------------------------------------
static void shtdPlk(void)
{
    ctrlk_exit();
    OPLK_FREE(drvInstance_l.pFileChunkBuffer);
    drvInstance_l.pFileChunkBuffer = NULL;
}

//------------------------------------------------------------------------------
/**
\brief    openPOWERLINK stack background tasks

This function runs the background tasks
*/
//------------------------------------------------------------------------------
static void bgtPlk(void)
{
    BOOL fExit = FALSE;

    while (1)
    {
        firmware_process();
        ctrlk_updateHeartbeat();
        fExit = ctrlk_process();

        if (fExit != FALSE)
            break;

        if (prodtest_process() != 0)
            break;
    }
}

//------------------------------------------------------------------------------
/**
\brief    Ctrl command execution callback

This function is called by the ctrlk module before executing the given command.
It only implements the supported commands.

\param  cmd_p               The command to be executed.
\param  pRet_p              Pointer to store the return value.
\param  pStatus_p           Pointer to store the kernel stack status. (if not NULL)
\param  pfExit_p            Pointer to store the exit flag. (if not NULL)

\return The function returns a BOOL.
\retval TRUE                Execution completed in callback.
\retval FALSE               Execution needed in ctrlk module.
*/
//------------------------------------------------------------------------------
static BOOL ctrlCommandExecCb(tCtrlCmdType cmd_p, UINT16* pRet_p, UINT16* pStatus_p,
                              BOOL* pfExit_p)
{
    tOplkError      retVal = kErrorOk;
    UINT16          status = kCtrlStatusUnchanged;
    BOOL            fExit = FALSE;

    switch (cmd_p)
    {
        case kCtrlInitStack:
            // Shutdown production test module before initializing the stack
            prodtest_exit();

            // Get init parameter and exchange MAC address
            {
                tCtrlInitParam  initParam;
                UINT8           aMacAddr[6];
                BOOL            fMacAddrValid;

                if (ctrlkcal_readInitParam(&initParam) != kErrorOk)
                    return FALSE;

                fMacAddrValid = (getMacAddress(aMacAddr) == kErrorOk);

                // Only exchange zero MAC address with valid MAC
                if (MAC_IS_ZERO(initParam.aMacAddress) && fMacAddrValid)
                {
                    OPLK_MEMCPY(initParam.aMacAddress, aMacAddr, 6);
                    ctrlkcal_storeInitParam(&initParam);
                }
            }

            drvInstance_l.fStackInitialized = TRUE;

            return FALSE;

        case kCtrlCleanupStack:
        case kCtrlShutdown:
            if (drvInstance_l.fStackInitialized)
            {
                drvInstance_l.fStackInitialized = FALSE;
                return FALSE;
            }

            // Stack has not been initialized, thus skip command execution in
            // ctrlk module. But report back to the user that kernel stack is
            // shut down successfully.
            retVal = kErrorOk;
            *pRet_p = (UINT16)retVal;

            if (cmd_p == kCtrlShutdown)
            {
                status = kCtrlStatusUnavailable;
                fExit = TRUE;
            }
            else
            {
                status = kCtrlStatusReady;
                fExit = FALSE;
            }

            break;

        case kCtrlWriteFileChunk:
            *pRet_p = writeFileChunk();
            status = kCtrlStatusUnchanged;
            fExit = FALSE;
            break;

        case kCtrlReconfigFactoryImage:
            retVal = setNextReconfigFirmware(kFirmwareImageFactory);
            *pRet_p = (UINT16)retVal;
            status = kCtrlStatusUnchanged;
            fExit = (retVal == kErrorOk); // Shutdown if image is okay
            break;

        case kCtrlReconfigUpdateImage:
            retVal = setNextReconfigFirmware(kFirmwareImageUpdate);
            *pRet_p = (UINT16)retVal;
            status = kCtrlStatusUnchanged;
            fExit = (retVal == kErrorOk); // Shutdown if image is okay
            break;

        default:
            return FALSE; // Command execution not implemented
    }

    if (pStatus_p != NULL)
        *pStatus_p = status;

    if (pfExit_p != NULL)
        *pfExit_p = fExit;

    return TRUE;
}

//------------------------------------------------------------------------------
/**
\brief  Write file chunk

This function handles the kCtrlWriteFileChunk command. It reads the file chunk
buffer and writes the data to the firmware update region in flash.

\return This function returns tOplkError error codes.
*/
//------------------------------------------------------------------------------
static tOplkError writeFileChunk(void)
{
    tOplkError              ret;
    INT                     retflash;
    tFlashInfo*             pFlashInfo = &drvInstance_l.flashInfo;
    UINT32                  updateImageOffset = firmware_getImageBase(kFirmwareImageUpdate);
    UINT32                  writeOffset;
    tOplkApiFileChunkDesc   fileChunkDesc;

    ret = ctrlk_readFileChunk(&fileChunkDesc, drvInstance_l.fileChunkBufferSize,
                              drvInstance_l.pFileChunkBuffer);
    if (ret != kErrorOk)
        return ret;

    // Check if the transfer starts correctly
    if (fileChunkDesc.fFirst && fileChunkDesc.offset != 0)
        return kErrorInvalidOperation;

    // Get write offset within flash
    writeOffset = updateImageOffset + fileChunkDesc.offset;

    // Check if write is done continuously
    if (!fileChunkDesc.fFirst && writeOffset != drvInstance_l.writeOffset)
        return kErrorInvalidOperation;

    // Check if write exceeds flash size
    if ((writeOffset + fileChunkDesc.length) > pFlashInfo->size)
        return kErrorNoResource;

    // Handle first transfer
    if (fileChunkDesc.fFirst)
    {
        // Reset write pointer
        drvInstance_l.writeOffset = writeOffset;

        // Erase first sector
        retflash = flash_eraseSector(updateImageOffset);
        if (retflash != 0)
            return kErrorGeneralError;

        // Set next sector to be erased
        drvInstance_l.writeEraseOffset = updateImageOffset + pFlashInfo->sectorSize;
    }

    // Handle sector boundary xing
    if ((writeOffset + fileChunkDesc.length) > drvInstance_l.writeEraseOffset)
    {
        // Chunk exceeds current sector -> erase next sector
        if (flash_eraseSector(drvInstance_l.writeEraseOffset) != 0)
            return kErrorGeneralError;

        drvInstance_l.writeEraseOffset += pFlashInfo->sectorSize;
    }

    // Forward data to flash
    retflash = flash_write(drvInstance_l.writeOffset, drvInstance_l.pFileChunkBuffer,
                           fileChunkDesc.length);
    if (retflash != 0)
        return kErrorGeneralError;

    drvInstance_l.writeOffset += fileChunkDesc.length;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Set next reconfigure firmware type

This function sets the reconfiguration to a valid firmware image.

\note   Note that the reconfiguration itself may only be triggered after the
        stack has been shutdown.

\param  imageType_p         Firmware image type to be reconfigured

\return This function returns tOplkError error codes.
*/
//------------------------------------------------------------------------------
static tOplkError setNextReconfigFirmware(tFirmwareImageType imageType_p)
{
    tOplkError      ret;

    switch (imageType_p)
    {
        case kFirmwareImageFactory:
            drvInstance_l.nextImage = imageType_p;
            return kErrorOk;

        case kFirmwareImageUpdate:
            // Update image must be verified before doing reconfiguration!
            break;

        default:
            return kErrorInvalidOperation;
    }

    ret = checkUpdateImage();
    if (ret != kErrorOk)
        return ret;

    drvInstance_l.nextImage = imageType_p;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Check update image

This function checks the update image header and the update image itself.

\return This function returns tOplkError error codes.
*/
//------------------------------------------------------------------------------
static tOplkError checkUpdateImage(void)
{
    UINT8           aBuffer[8 * 1024];
    tFirmwareHeader firmwareHeader;
    UINT32          crcVal;
    UINT            i;
    UINT            length;
    UINT32          offset;

    if (flash_read(firmware_getImageBase(kFirmwareImageUpdate),
       (UINT8*)&firmwareHeader, sizeof(tFirmwareHeader)) != 0)
    {
        return kErrorNoResource;
    }

    PRINTF("Firmware header:\n");
    PRINTF(" Signature      0x%08X (0x%08X)\n", firmwareHeader.signature, FIRMWARE_HEADER_SIGNATUR);
    PRINTF(" Version        0x%08X (0x%08X)\n", firmwareHeader.version, FIRMWARE_HEADER_VERSION);
    PRINTF(" Time stamp     0x%08X\n", firmwareHeader.timeStamp);
    PRINTF(" Length         0x%08X\n", firmwareHeader.length);
    PRINTF(" CRC            0x%08X\n", firmwareHeader.crc);
    PRINTF(" OPLK Version   0x%08X\n", firmwareHeader.oplkVersion);
    PRINTF(" OPLK Feature   0x%08X\n", firmwareHeader.oplkFeature);
    PRINTF(" Header CRC     0x%08X\n", firmwareHeader.headerCrc);

    if (firmware_checkHeader(&firmwareHeader) != 0)
        return kErrorGeneralError;

    PRINTF("Calc image CRC...\n");

    i = firmwareHeader.length;
    offset = firmware_getImageBase(kFirmwareImageUpdate) + sizeof(tFirmwareHeader);
    crcVal = 0xFFFFFFFF;

    while (i > 0)
    {
        if (i > sizeof(aBuffer))
            length = sizeof(aBuffer);
        else
            length = i;

        if (flash_read(offset, aBuffer, length) != 0)
            return kErrorNoResource;

        firmware_calcCrc(&crcVal, aBuffer, length);

        i -= length;
        offset += length;
    }

    PRINTF("Calculated Image CRC = 0x%08X\n", crcVal);

    if (crcVal != firmwareHeader.crc)
    {
        PRINTF(" --> Wrong CRC!\n");
        return kErrorGeneralError;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Get MAC address

This function reads the MAC address from the flash and writes it to the given
address.

\param  pMacAddr_p      Pointer to memory where the MAC address is returned.

\return This function returns tOplkError error codes.
*/
//------------------------------------------------------------------------------
static tOplkError getMacAddress(UINT8* pMacAddr_p)
{
    UINT32                  offset;
    tFirmwareDeviceHeader   deviceHeader;

    if (pMacAddr_p == NULL)
        return kErrorGeneralError;

    offset = firmware_getDeviceHeaderBase();

    if (offset == FIRMWARE_INVALID_IMAGE_BASE)
        return kErrorGeneralError;

    if (flash_read(offset, (UINT8*)&deviceHeader, sizeof(tFirmwareDeviceHeader)) != 0)
        return kErrorGeneralError;

    if (firmware_checkDeviceHeader(&deviceHeader) != 0)
        return kErrorGeneralError;

    OPLK_MEMCPY(pMacAddr_p, deviceHeader.aMacAddr, 6);

    return kErrorOk;
}

/// \}

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

#include <flash.h>
#include <firmware.h>

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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    tFlashInfo          flashInfo;          ///< Flash info
    UINT32              writeOffset;        ///< Current flash write offset
    UINT32              writeEraseOffset;   ///< Current flash erase offset
    tFirmwareImageType  nextImage;          ///< Next firmware image to be configured

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
static tOplkError writeUpdateImage(tCtrlDataChunk* pDataChunk_p);
static tOplkError setNextReconfigFirmware(tFirmwareImageType imageType_p);
static tOplkError checkUpdateImage(void);

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
                PRINTF("Firmware in factory image mode\n");
                // Try to boot update image only after por!
                if (firmware_getStatus() == kFirmwareStatusPor)
                {
                    PRINTF(" -> Came from POR, check for valid update image...\n");
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
                break;

            case kFirmwareImageUpdate:
                PRINTF("Firmware in update image mode\n");
                // We are in update image, nothing to do...
            default:
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
    UINT16          status;
    BOOL            fExit;
    tCtrlDataChunk  dataChunk;

    switch (cmd_p)
    {
        case kCtrlWriteFile:
            retVal = ctrlk_getFileTransferChunk(&dataChunk);
            if (retVal != kErrorOk)
            {
                *pRet_p = (UINT16)retVal;
                fExit = TRUE;
                break;
            }

            if (dataChunk.fileType == kCtrlFileTypeFirmwareUpdate)
                retVal = writeUpdateImage(&dataChunk);
            else
                retVal = kErrorGeneralError;

            *pRet_p = (UINT16)retVal;
            fExit = FALSE;
            break;

        case kCtrlSetKernelFactoryImage:
            retVal = setNextReconfigFirmware(kFirmwareImageFactory);
            *pRet_p = (UINT16)retVal;
            fExit = FALSE;
            break;

        case kCtrlSetKernelUpdateImage:
            retVal = setNextReconfigFirmware(kFirmwareImageUpdate);
            *pRet_p = (UINT16)retVal;
            fExit = FALSE;
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
\brief    Write update image chunk to flash

This function writes the update image data chunk to the flash. The provided
data chunk has to be read from the ctrl module's transfer buffer.

\param  pDataChunk_p        Data chunk information

\return This function returns tOplkError error codes.
*/
//------------------------------------------------------------------------------
static tOplkError writeUpdateImage(tCtrlDataChunk* pDataChunk_p)
{
    tOplkError          ret;
    int                 retFlash;
    tFlashInfo*         pFlashInfo = &drvInstance_l.flashInfo;
    UINT32              updateImageOffset = firmware_getImageBase(kFirmwareImageUpdate);
    UINT32              writeOffset;
    UINT8               aBuffer[CTRL_FILETRANSFER_SIZE];

    if (pDataChunk_p->length > sizeof(aBuffer))
        return kErrorNoResource;

    ret = ctrlk_readFileTransfer(sizeof(aBuffer), aBuffer);
    if (ret != kErrorOk)
        return ret;

    // Check start condition
    if (pDataChunk_p->fStart && (pDataChunk_p->offset != 0))
        return kErrorInvalidOperation;

    // Get offset within flash
    writeOffset = updateImageOffset + pDataChunk_p->offset;

    // Check if continuous write is done
    if (!pDataChunk_p->fStart && (writeOffset != drvInstance_l.writeOffset))
        return kErrorInvalidOperation; // Command skips some data, shouldn't be!

    // Check if write exceeds flash size
    if ((writeOffset + pDataChunk_p->length) > pFlashInfo->size)
        return kErrorGeneralError; // Image exceeds flash size!

    // Handle start
    if (pDataChunk_p->fStart)
    {
        // Reset write pointer
        drvInstance_l.writeOffset = writeOffset;

        // Erase first sector
        retFlash = flash_eraseSector(updateImageOffset);
        if (retFlash != 0)
            return kErrorGeneralError;

        // Set next sector to be erased
        drvInstance_l.writeEraseOffset = updateImageOffset + pFlashInfo->sectorSize;
    }

    // Handle sector boundary crossing
    if ((writeOffset + pDataChunk_p->length) > drvInstance_l.writeEraseOffset)
    {
        // Chunk exceeds current sector => erase next sector
        if (flash_eraseSector(drvInstance_l.writeEraseOffset) != 0)
            return kErrorGeneralError;

        drvInstance_l.writeEraseOffset += pFlashInfo->sectorSize;
    }

    // Forward data to flash
    retFlash = flash_write(drvInstance_l.writeOffset, aBuffer, pDataChunk_p->length);
    if (retFlash != 0)
        return kErrorGeneralError;

    // At this point data is forwarded to the flash
    drvInstance_l.writeOffset += pDataChunk_p->length;

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

/// \}

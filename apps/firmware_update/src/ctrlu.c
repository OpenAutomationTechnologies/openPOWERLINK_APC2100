/**
********************************************************************************
\file   ctrlu.c

\brief  User stack control module for firmware update

This file contains the implementation of the user stack control module.

\ingroup module_ctrlu
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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
#include <common/oplkinc.h>
#include <ctrlu.h>
#include <user/ctrlucal.h>
#include <common/target.h>

#include <stddef.h>
#include <limits.h>

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
    UINT16              lastHeartbeat;          ///< Last detected heartbeat
    tOplkApiInitParam   initParam;              ///< Stack initialization parameters
    tCtrlKernelInfo     kernelInfo;             ///< Information about kernel stack
    UINT32              requiredKernelFeatures; ///< Kernel stack features we need to run correctly
    BOOL                fInitialized;           ///< Flag determines if stack is initialized/ready
} tCtrluInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tCtrluInstance       ctrlInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize user control module

The function initializes the user control module.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_init(void)
{
    tOplkError          ret;

    DEBUG_LVL_CTRL_TRACE("Initialize ctrl module ...\n");

    ctrlInstance_l.lastHeartbeat = 0;

    if ((ret = ctrlucal_init()) != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Could not initialize ctrlucal\n");
        goto Exit;
    }

    if ((ret = ctrlucal_checkKernelStack()) != kErrorOk)
    {
        ctrlucal_exit();
        goto Exit;
    }

    if ((ret = ctrlu_getKernelInfo(&ctrlInstance_l.kernelInfo)) != kErrorOk)
    {
        ctrlucal_exit();
        goto Exit;
    }

    if (ctrlInstance_l.kernelInfo.version == PLK_DEFINED_STACK_VERSION)
    {
        DEBUG_LVL_ALWAYS_TRACE("Kernel features: 0x%08x\n", ctrlInstance_l.kernelInfo.featureFlags);
        DEBUG_LVL_ALWAYS_TRACE("Kernel version: 0x%08x\n", ctrlInstance_l.kernelInfo.version);
        return kErrorOk;
    }
    else
    {
        DEBUG_LVL_ERROR_TRACE("Kernel feature/version mismatch:\n");
        DEBUG_LVL_ERROR_TRACE("  Version: Is:%08x - required:%08x\n",
                              ctrlInstance_l.kernelInfo.version, PLK_DEFINED_STACK_VERSION);
        DEBUG_LVL_ERROR_TRACE("  Features: Is:%08x - required:%08x\n",
                              ctrlInstance_l.kernelInfo.featureFlags, ctrlInstance_l.requiredKernelFeatures);
        ctrlucal_exit();
        return kErrorFeatureMismatch;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup user control module

The function cleans up the user control module.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
void ctrlu_exit(void)
{
    ctrlucal_exit();
}

//------------------------------------------------------------------------------
/**
\brief  Initialize openPOWERLINK stack

The function initializes the openPOWERLINK stack. It initializes all
user modules and communication with the kernel control module to initialize
the kernel modules.

After returning from this function, the application must start the NMT state
machine via oplk_execNmtCommand(kNmtEventSwReset) and thereby the whole
openPOWERLINK stack.

\param  pInitParam_p            Pointer to the initialization parameters
                                provided by the application.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_initStack(tOplkApiInitParam* pInitParam_p)
{
    tOplkError              ret = kErrorOk;
    tCtrlInitParam          ctrlParam;
    UINT16                  retVal;

    // reset instance structure
    OPLK_MEMSET(&ctrlInstance_l.initParam, 0, sizeof(tOplkApiInitParam));
    OPLK_MEMCPY(&ctrlInstance_l.initParam, pInitParam_p,
                min(sizeof(tOplkApiInitParam), (size_t)pInitParam_p->sizeOfInitParam));

    DEBUG_LVL_CTRL_TRACE("Initializing kernel modules ...\n");
    OPLK_MEMCPY(ctrlParam.aMacAddress, ctrlInstance_l.initParam.aMacAddress, 6);
    strncpy(ctrlParam.szEthDevName, ctrlInstance_l.initParam.hwParam.pDevName, 127);
    ctrlParam.ethDevNumber = ctrlInstance_l.initParam.hwParam.devNum;
    ctrlucal_storeInitParam(&ctrlParam);

    if ((ret = ctrlucal_executeCmd(kCtrlInitStack, &retVal)) != kErrorOk)
        goto Exit;

    if ((tOplkError)retVal != kErrorOk)
    {
        ret = (tOplkError)retVal;
        goto Exit;
    }

    /* Read back init param because current MAC address was copied by DLLK */
    ret = ctrlucal_readInitParam(&ctrlParam);
    if (ret != kErrorOk)
    {
        goto Exit;
    }

    OPLK_MEMCPY(ctrlInstance_l.initParam.aMacAddress, ctrlParam.aMacAddress, 6);
    // the application must start NMT state machine
    // via oplk_execNmtCommand(kNmtEventSwReset)
    // and thereby the whole POWERLINK stack

    ctrlInstance_l.fInitialized = TRUE;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown openPOWERLINK stack

The function shuts down the openPOWERLINK stack. I cleans up all user modules
and the kernel modules by using the kernel control module.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_shutdownStack(void)
{
    tOplkError      ret = kErrorOk;
    UINT16          retVal;

    ctrlInstance_l.fInitialized = FALSE;

    /* shutdown kernel stack */
    ret = ctrlucal_executeCmd(kCtrlShutdown, &retVal);
    DEBUG_LVL_CTRL_TRACE("shoutdown kernel modules():  0x%X\n", ret);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get information about kernel stack

The function gets information about the version and features of the kernel stack.

\param  pKernelInfo_p       Pointer to store kernel information.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_getKernelInfo(tCtrlKernelInfo* pKernelInfo_p)
{
    UINT16      retVal;

    if (ctrlucal_executeCmd(kCtrlGetFeaturesHigh, &retVal) != kErrorOk)
        return kErrorNoResource;
    pKernelInfo_p->featureFlags = (retVal << 16);

    if (ctrlucal_executeCmd(kCtrlGetFeaturesLow, &retVal) != kErrorOk)
        return kErrorNoResource;
    pKernelInfo_p->featureFlags |= retVal;

    if (ctrlucal_executeCmd(kCtrlGetVersionHigh, &retVal) != kErrorOk)
        return kErrorNoResource;
    pKernelInfo_p->version = (retVal << 16);

    if (ctrlucal_executeCmd(kCtrlGetVersionLow, &retVal) != kErrorOk)
        return kErrorNoResource;
    pKernelInfo_p->version |= retVal;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get Ethernet Interface MAC address

The function returns the Ethernet Interface MAC address used by the
Ethernet controller.

\return The function returns the Ethernet MAC address.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
UINT8* ctrlu_getEthMacAddr(void)
{
    return &ctrlInstance_l.initParam.aMacAddress[0];
}

//------------------------------------------------------------------------------
/**
\brief  Returns the stacks initialization state

The function returns the initialization state of the stack.

\return The function returns TRUE if the stack is initialized and running or
        FALSE if it is shutdown.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
BOOL ctrlu_stackIsInitialized(void)
{
    return ctrlInstance_l.fInitialized;
}

//------------------------------------------------------------------------------
/**
\brief  Write file to kernel stack

The function writes the given file to the kernel layer.

\param  type_p              Select file type to be written
\param  length_p            Length of given file
\param  pBuffer_p           Pointer to file buffer

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_writeFileToKernel(tCtrlFileType type_p, INT length_p, UINT8* pBuffer_p)
{
    tOplkError      ret = kErrorOk;
    UINT16          retVal;
    tCtrlDataChunk  dataChunk;
    INT             length = length_p;
    float           completePer = 0;
    float           completedLength, initlength = (float)(length_p);

    if ((pBuffer_p == NULL) || (length_p <= 0))
        return kErrorInvalidOperation;

    OPLK_MEMSET((void*)&dataChunk, 0, sizeof(tCtrlDataChunk));

    DEBUG_LVL_CTRL_TRACE("%s Start write to kernel stack...\n", __func__);

    // Signalize start of file image
    dataChunk.offset = 0;
    dataChunk.fStart = 1;

    dataChunk.fileType = type_p;

    do
    {
        if (length < CTRL_FILETRANSFER_SIZE)
        {
            dataChunk.length = length;
            dataChunk.fLast = 1;
        }
        else
        {
            dataChunk.length = CTRL_FILETRANSFER_SIZE;
            dataChunk.fLast = 0;
        }

        ret = ctrlucal_setFileTransferChunk(&dataChunk);
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("Setting file transfer chunk failed with 0x%X\n", ret);
            goto Exit;
        }

        ret = ctrlucal_storeFileTransfer(dataChunk.length, pBuffer_p + dataChunk.offset);
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("Storing file transfer chunk failed with 0x%X\n", ret);
            goto Exit;
        }

        ret = ctrlucal_executeCmd(kCtrlWriteFile, &retVal);
        if (ret != kErrorOk || (tOplkError)retVal != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("%s Exec write file to kernel failed\n", __func__);
            DEBUG_LVL_ERROR_TRACE(" Command ret: 0x%X\n", ret);
            DEBUG_LVL_ERROR_TRACE(" Kernel ret:  0x%X\n", retVal);

            // Return kernel error if user is ok
            ret = (ret == kErrorOk) ? (tOplkError)retVal : ret;
            goto Exit;
        }

        dataChunk.fStart = 0; // First chunk is done for sure

        // Switch to next file chunk
        dataChunk.offset += dataChunk.length;
        length -= dataChunk.length;

#if (DEBUG_GLB_LVL & DEBUG_LVL_CTRL)
        {
            static UINT8    lastProg = 0;
            UINT8           currentProg = (dataChunk.offset * 100) / length_p;

            if (currentProg > lastProg)
            {
                TRACE(" Downloading %08X (%3d %%)\n", dataChunk.offset, currentProg);
                lastProg = currentProg;
            }
        }
#endif
        // Display progress of download
        completedLength = (float) length;
        completePer = (((initlength - completedLength) / initlength) * 100.0);
        printf("\rProgress [%.0f%%]", floor(completePer));
        fflush(stdout);
    } while (length > 0);
    printf("\n");

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Sets the next firmware image flag in the kernel

The function sets the next firmware image flag to be used for reconfguring the
device on exit.

\return The function returns the kernel feature flags.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_setNextImageFlag(tCtrlFileType fileType_p)
{
    tOplkError      ret;
    tCtrlCmdType    cmd;
    UINT16          retVal;

    if (fileType_p == kCtrlFileTypeFirmwareUpdate)
    {
        cmd = kCtrlSetKernelUpdateImage;
        ret = ctrlucal_executeCmd(cmd, &retVal);
    }
    else
    {
        cmd = kCtrlSetKernelFactoryImage;
        ret = ctrlucal_executeCmd(cmd, &retVal);
    }

    if ((ret != kErrorOk) || ((tOplkError) retVal != kErrorOk))
    {
        printf("Exec command %d failed\n", cmd);
        printf(" Kernel : 0x%X\n", retVal);
        printf(" User   : 0x%X\n", ret);
    }

    return (tOplkError)retVal;
}
//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}

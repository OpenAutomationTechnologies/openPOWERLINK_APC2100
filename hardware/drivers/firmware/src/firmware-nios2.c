/**
********************************************************************************
\file   firmware-nios2.c

\brief  Firmware driver for Nios II

This file implements the firmware driver for Nios II using the Remote Update
Core for Cyclone III/IV integrated in Qsys.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <firmware.h>
#include <oplk/oplk.h>

#include <system.h>
#include <io.h>
#include <stdlib.h>

// Check if system.h provides the REMOTE_UPDATE parameters.
#if defined(REMOTE_UPDATE_BASE)
#include <firmware-info.h>  // Since there is remote update core, also the info
                            // header is required then!

#else

#endif

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

#define FIRMWARE_IO_RD(offset)          IORD(REMOTE_UPDATE_BASE, offset)
#define FIRMWARE_IO_WR(offset, val)     IOWR(REMOTE_UPDATE_BASE, offset, val)

#define FIRMWARE_MSM_MODE_MASK          0x3
#define FIRMWARE_MSM_MODE_FACTORY       0x0
#define FIRMWARE_MSM_MODE_APP           0x1
#define FIRMWARE_MSM_MODE_WDOG          0x2

#define FIRMWARE_RECONF_DUE_MASK        0x1F
#define FIRMWARE_RECONF_DUE_POWERUP     0x00
#define FIRMWARE_RECONF_DUE_LOGIC       0x01
#define FIRMWARE_RECONF_DUE_WDOG        0x02
#define FIRMWARE_RECONF_DUE_NSTATUS     0x04
#define FIRMWARE_RECONF_DUE_CRC         0x08
#define FIRMWARE_RECONF_DUE_NCONFIG     0x10

#define FIRMWARE_WDOG_MAXVALUE          0xFFF
#define FIRMWARE_WDOG_RESET             0x02

#define FIRMWARE_RECONFIG               0x01

// Invalidate not set image base addresses
#ifndef FIRMWARE_FACTORY_IMAGE_BASE
#define FIRMWARE_FACTORY_IMAGE_BASE     FIRMWARE_INVALID_IMAGE_BASE
#endif

#ifndef FIRMWARE_UPDATE_IMAGE_BASE
#define FIRMWARE_UPDATE_IMAGE_BASE      FIRMWARE_INVALID_IMAGE_BASE
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
*  \brief Firmware instance
*
*  The struct defines the Firmware instance.
*/
typedef struct
{
    BOOL                fInitialized;   ///< Initialization flag for firmware module
    BOOL                fResetWdog;     ///< Reset Watchdog to avoid reconfig

} tFirmwareInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tFirmwareInstance firmwareInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static UINT32 getMsmCurrentState(void);

static void setWdogEnable(BOOL fEnable_p);
static BOOL getWdogEnable(void);
static void setWdogTimeout(UINT32 timeout_p);
static void resetWdog(void);

static void setFactoryReconfig(void);
static void setReconfigAddr(UINT32 addr_p);

static UINT32 getReconfigCondition(UINT past_p);
static UINT32 getBootAddress(UINT past_p);
static void triggerReconfig(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize Firmware module

The function initializes the Firmware module before being used.

\return The function returns 0 if the Firmware module has been initialized
        successfully, otherwise -1.
*/
//------------------------------------------------------------------------------
int firmware_init(void)
{
    int ret = 0;

    memset((void*)&firmwareInstance_l, 0, sizeof(tFirmwareInstance));

    firmwareInstance_l.fResetWdog = getWdogEnable();
    firmwareInstance_l.fInitialized = TRUE;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Exit Firmware module

The function shuts down the Firmware module.
*/
//------------------------------------------------------------------------------
void firmware_exit(void)
{
    firmwareInstance_l.fInitialized = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Get current image type

The function provides the currently loaded image type.

\return The function returns the current image type.
*/
//------------------------------------------------------------------------------
tFirmwareImageType firmware_getCurrentImageType(void)
{
    tFirmwareImageType  imageType;

    switch (getMsmCurrentState())
    {
        case FIRMWARE_MSM_MODE_FACTORY:
            imageType = kFirmwareImageFactory;
            break;

        case FIRMWARE_MSM_MODE_APP:
        case (FIRMWARE_MSM_MODE_APP | FIRMWARE_MSM_MODE_WDOG):
            imageType = kFirmwareImageUpdate;
            break;

        default:
            imageType = kFirmwareImageUnknown;
            break;
    }

    return imageType;
}

//------------------------------------------------------------------------------
/**
\brief  Get current image status

The function returns the status of the current image.

\return The function returns the current image status.
*/
//------------------------------------------------------------------------------
tFirmwareStatus firmware_getStatus(void)
{
    tFirmwareStatus     status;
    UINT32              val;

    if (getMsmCurrentState() == FIRMWARE_MSM_MODE_FACTORY)
    {
        val = getReconfigCondition(1);

        // One-hot coded, however higher precedes lower bit
        if (val & FIRMWARE_RECONF_DUE_NCONFIG)
            status = kFirmwareStatusReconfig;
        else if (val & FIRMWARE_RECONF_DUE_CRC)
            status = kFirmwareStatusError;
        else if (val & FIRMWARE_RECONF_DUE_NSTATUS)
            status = kFirmwareStatusReconfig;
        else if (val & FIRMWARE_RECONF_DUE_WDOG)
        {
            // Consider workaround to reconfig from factory to factory!
            // -> Then watchdog is not an error, just reconfiguration.
            if (getBootAddress(1) == FIRMWARE_FACTORY_IMAGE_BASE)
                status = kFirmwareStatusReconfig;
            else
                status = kFirmwareStatusError;
        }
        else if (val & FIRMWARE_RECONF_DUE_LOGIC)
            status = kFirmwareStatusReconfig;
        else //FIRMWARE_RECONF_DUE_POWERUP
            status = kFirmwareStatusPor;
    }
    else
    {
        // In update image state the condition register cannot be read.
        // Simply return reconfig status.
        status = kFirmwareStatusReconfig;
    }

    return status;
}

//------------------------------------------------------------------------------
/**
\brief  Get image base address

The function returns the base address of the selected image.

\param  sel_p       Selects the image which base address shall be returned.

\return The function returns the selected image's base address.
\retval FIRMWARE_INVALID_IMAGE_BASE     If the image's base address is not
                                        specified in firmware-info.h.
*/
//------------------------------------------------------------------------------
UINT32 firmware_getImageBase(tFirmwareImageType sel_p)
{
    // Simply return addresses specified in firmware-info.h
    switch (sel_p)
    {
    case kFirmwareImageFactory:
        return FIRMWARE_FACTORY_IMAGE_BASE;

    case kFirmwareImageUpdate:
        return FIRMWARE_UPDATE_IMAGE_BASE;

    default:
        return FIRMWARE_INVALID_IMAGE_BASE;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Calculate CRC

The function calculates the CRC value of the given buffer. The caller must
provide a 32 bit buffer which is used to calculate the CRC chunk-wise.
The caller must initialize the buffer to 0xFFFFFFFF.

\note   This implementation bases on AN458.

\param  pCrcVal_p   Buffer for CRC value calculation
\param  pBuffer_p   Pointer to buffer of chunk data for CRC calculation
\param  length_p    Length of buffer in byte

The function returns 0 if the CRC calculation was successful, otherwise -1.
*/
//------------------------------------------------------------------------------
int firmware_calcCrc(UINT32* pCrcVal_p, UINT8* pBuffer_p, INT length_p)
{
    UINT32  crcval;
    UINT8   cval;
    int     i;

    if (pCrcVal_p == NULL)
        return -1;

    crcval = *pCrcVal_p;

    for (; length_p; length_p--)
    {
        cval = *pBuffer_p;
        crcval ^= cval;

        for (i=8; i; i--)
            crcval = (crcval & 0x00000001) ? ((crcval >> 1) ^ 0xEDB88320) : (crcval >> 1);

        pBuffer_p++;
    }

    *pCrcVal_p = crcval;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Firmware process function

This is the firmware process function, which shall be called on a regular basis.

*/
//------------------------------------------------------------------------------
void firmware_process(void)
{
    if (firmwareInstance_l.fResetWdog)
        resetWdog();
}

//------------------------------------------------------------------------------
/**
\brief  Trigger firmware reconfiguration

This function triggers firmware reconfiguration to the specified image.

\param  next_p      Sets the image which shall be loaded next.

\note   The function never returns, because it triggers Firmware reconfiguration.

*/
//------------------------------------------------------------------------------
void firmware_reconfig(tFirmwareImageType next_p)
{
    if (!(getMsmCurrentState() & FIRMWARE_MSM_MODE_APP))
    {
        if (next_p == kFirmwareImageUpdate)
        {
            setFactoryReconfig();
            // Reconfig from image but add header offset!
            setReconfigAddr(FIRMWARE_UPDATE_IMAGE_BASE + sizeof(tFirmwareHeader));
            setWdogTimeout(FIRMWARE_WDOG_TIMEOUT);
            setWdogEnable(FIRMWARE_WDOG_ENABLE);
        }
        else if (next_p == kFirmwareImageFactory)
        {
            setFactoryReconfig();
            setReconfigAddr(FIRMWARE_FACTORY_IMAGE_BASE);
            setWdogTimeout(1); // Set very short timeout, to force fallback.
            setWdogEnable(TRUE);
        }
    }

    triggerReconfig();

    while (1);  // You should never come this far. For debugging purpose use
                // this line setting a breakpoint or so.
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Get Remote Update Core Master State Machine

This function returns the current Master State Machine mode.
It identifies the currently loaded image type (factory or update image) and
informs in update image mode if the watchdog is enabled.

\return The function returns the Master State Machine mode.
\retval FIRMWARE_MSM_MODE_FACTORY   This bit identifies the factory image mode.
\retval FIRMWARE_MSM_MODE_APP       This bit identifies the update image mode.
                                    This bit could be returned combined with
                                    FIRMWARE_MSM_MODE_WDOG.
\retval FIRMWARE_MSM_MODE_WDOG      This bit identifies that the watchdog is
                                    enabled. This bit is returned combined with
                                    FIRMWARE_MSM_MODE_APP.

*/
//------------------------------------------------------------------------------
static UINT32 getMsmCurrentState(void)
{
    return (FIRMWARE_IO_RD(0x00) & FIRMWARE_MSM_MODE_MASK);
}

//------------------------------------------------------------------------------
/**
\brief  Set Remote Update Core Master watchdog enable

This function set the watchdog enable control.

\param  fEnable_p   Control watchdog enable bit. TRUE enables the watchdog,
                    FALSE disables the watchdog.

\note   The watchdog can only be enabled/disabled in factory image mode.
        If this function is called in other modes, the call has no effect.
*/
//------------------------------------------------------------------------------
static void setWdogEnable(BOOL fEnable_p)
{
    // Only factory mode works here!
    if (getMsmCurrentState() != FIRMWARE_MSM_MODE_FACTORY)
        return;

    FIRMWARE_IO_WR(0x03, (fEnable_p) ? 1 : 0);
}

//------------------------------------------------------------------------------
/**
\brief  Get Remote Update Core Master watchdog enable

This function returns the watchdog enable bit state.

\return The function returns a BOOL that identifies the enable bit state.
\retval TRUE        The watchdog is enabled.
\retval FALSE       The watchdog is disabled.
*/
//------------------------------------------------------------------------------
static BOOL getWdogEnable(void)
{
    UINT32 msmState = getMsmCurrentState();

    // In application mode the wdog enable bit is provided in MSM state
    if (msmState & FIRMWARE_MSM_MODE_APP)
    {
        return (msmState & FIRMWARE_MSM_MODE_WDOG);
    }

    // In factory mode the wdog enable bit is in extra register
    return FIRMWARE_IO_RD(0x1B);
}

//------------------------------------------------------------------------------
/**
\brief  Set Remote Update Core Master watchdog timeout

This function set the watchdog timeout.

\param  timeout_p   The watchdog timeout to be set to the Remote Update Core.
                    Note that the timeout is clipped to the maximum value
                    FIRMWARE_WDOG_MAXVALUE.

\note   The watchdog timeout can only be set in factory image mode.
        If this function is called in other modes, the call has no effect.
*/
//------------------------------------------------------------------------------
static void setWdogTimeout(UINT32 timeout_p)
{
    UINT32 timeout;

    // Only factory mode works here!
    if (getMsmCurrentState() != FIRMWARE_MSM_MODE_FACTORY)
        return;

    timeout = (timeout_p > FIRMWARE_WDOG_MAXVALUE) ?
              FIRMWARE_WDOG_MAXVALUE : /* Clip to maximum value */
              timeout_p; /* Set specified value */

    FIRMWARE_IO_WR(0x02, timeout);
}

//------------------------------------------------------------------------------
/**
\brief  Set necessary Remote Update Core reconfig flags

This function sets the needed reconfiguration flags "early config-done" and
"internal oscillator".

\note   The watchdog timeout can only be set in factory image mode.
        If this function is called in other modes, the call has no effect.
*/
//------------------------------------------------------------------------------
static void setFactoryReconfig(void) //TODO: Check if this shall be called in update image mode also!
{
    FIRMWARE_IO_WR(0x01, 1); // Early confdone
    FIRMWARE_IO_WR(0x06, 1); // Internal oscillator
}

//------------------------------------------------------------------------------
/**
\brief  Set reconfiguration address to Remote Update Core

This function sets the reconfiguration address register in Remote Update Core.

\param  addr_p      Reconfiguration image base address in configuration Flash.

*/
//------------------------------------------------------------------------------
static void setReconfigAddr(UINT32 addr_p) //TODO: Check if this call is valid in update mode also!
{
    FIRMWARE_IO_WR(0x04, (addr_p >> 2)); //TODO: Limit to 24 bit?
}

//------------------------------------------------------------------------------
/**
\brief  Reset Remote Update Core watchdog

This function resets the watchdog circuit in Remote Update Core.

*/
//------------------------------------------------------------------------------
static void resetWdog(void)
{
    UINT32  val;

    // Only with enabled wdog this call makes sense
    if (!(getMsmCurrentState() & FIRMWARE_MSM_MODE_WDOG)) //TODO: Shall we also return silently in factory image mode?
        return;

    val = FIRMWARE_IO_RD(0x20);

    val |= FIRMWARE_WDOG_RESET;
    FIRMWARE_IO_WR(0x20, val);

    //TODO: Do we need that sleep?
    usleep(1); // sleep to assure reset pulse

    val &= ~FIRMWARE_WDOG_RESET;
    FIRMWARE_IO_WR(0x20, val);
}

//------------------------------------------------------------------------------
/**
\brief  Get Remote Update Core trigger reconfiguration condition

This function returns the specified past trigger reconfiguration condition.

\param  past_p      This parameter specifies which register shall be read.
                    Allowed values are 1 and 2. Any other value is ignored.

\note   Note that accessing the trigger reconfiguration condition registers is
        only allowed in factory image mode!

\return The function returns the trigger reconfiguration condition register
        content.
\retval FIRMWARE_RECONF_DUE_POWERUP     Reconfiguration from power-up
\retval FIRMWARE_RECONF_DUE_LOGIC       Reconfiguration trigger from logic array
\retval FIRMWARE_RECONF_DUE_WDOG        Reconfiguration trigger due to watchdog
                                        timeout
\retval FIRMWARE_RECONF_DUE_NSTATUS     Reconfiguration due to nSTATUS pin
                                        assertion
\retval FIRMWARE_RECONF_DUE_CRC         Reconfiguration due to CRC error during
                                        configuration
\retval FIRMWARE_RECONF_DUE_NCONFIG     Reconfiguration due to nCONFIG pin
                                        assertion

\retval FIRMWARE_RECONF_DUE_MASK        This function was called in wrong MSM
                                        mode!
*/
//------------------------------------------------------------------------------
static UINT32 getReconfigCondition(UINT past_p)
{
    int     offset;

    // Only factory mode works here!
    if (getMsmCurrentState() != FIRMWARE_MSM_MODE_FACTORY)
        return FIRMWARE_RECONF_DUE_MASK;

    switch (past_p)
    {
        case 1:
            offset = 0x0F;
            break;

        case 2:
            offset = 0x17;
            break;

        default:
            return FIRMWARE_RECONF_DUE_MASK;
    }

    return (FIRMWARE_IO_RD(offset) & FIRMWARE_RECONF_DUE_MASK);
}

//------------------------------------------------------------------------------
/**
\brief  Get Remote Update Core boot addresses

This function returns the past boot addresses.

param   past_p      This parameter specifies which register shall be read.
                    Allowed values are 1 and 2. Any other value is ignored.

\note   Note that accessing the boot address registers is only allowed in
        factory image mode!

\return The function returns the boot address register selected with past_p.
\retval FIRMWARE_INVALID_IMAGE_BASE     This function was called in wrong MSM
                                        mode!
*/
//------------------------------------------------------------------------------
static UINT32 getBootAddress(UINT past_p)
{
    int     offset;

    // Only factory mode works here!
    if (getMsmCurrentState() != FIRMWARE_MSM_MODE_FACTORY)
        return FIRMWARE_INVALID_IMAGE_BASE;

    switch (past_p)
    {
        case 1:
            offset = 0x0C;
            break;

        case 2:
            offset = 0x14;
            break;

        default:
            return FIRMWARE_INVALID_IMAGE_BASE;
    }

    return FIRMWARE_IO_RD(offset);
}

//------------------------------------------------------------------------------
/**
\brief  Trigger reconfiguration by Remote Update Core

This function triggers the reconfiguration process in the Remote Update Core.

\note   Note that this function may not return if the reconfiguration process
        starts before the function return.

*/
//------------------------------------------------------------------------------
static void triggerReconfig(void)
{
    FIRMWARE_IO_WR(0x20, FIRMWARE_RECONFIG);
}

/// \}

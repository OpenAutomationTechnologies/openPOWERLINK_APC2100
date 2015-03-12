/**
********************************************************************************
\file   plkled-xpc2100.c

\brief  POWERLINK LED for APC2100/PPC2100

The file implements the POWERLINK LED on APC2100/PPC2100 used by openPOWERLINK
stack.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2015, Kalycito Infotech Private Ltd.All rights reserved.
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
#include <oplk/oplk.h>
#include <common/driver.h>
#include "plkled.h"

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
#define PLKLED_OUTSET_OFFSET              0x10
#define PLKLED_OUTCLEAR_OFFSET            0x14
#define PLKLED_STATUS_MASK                (1 << 0)
#define PLKLED_ERROR_MASK                 (1 << 1)
#define PLKLED_WRITE_REG(offset, val)     (*((volatile UINT32*)((plkLedInstance_l.pPlkLedReg + offset))) = (val))
#define PLKLED_READ_REG(offset)           (*((volatile UINT32*)((plkLedInstance_l.pPlkLedReg + offset))))
//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Local instance of POWERLINK LED module

The following structure defines a local parameters required by POWERLINK LED
module.
*/
typedef struct
{
    HANDLE    oplkFileHandle;    ///< File handle for openPOWERLINK driver.
    UINT8*    pPlkLedReg;        ///< Pointer to base address of POWERLINK LEDs.
} tPlkLedInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tPlkLedInstance     plkLedInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize GPIO module

The function initializes the GPIO module.
*/
//------------------------------------------------------------------------------
void plkled_init(void)
{
    UINT32      errCode;
    ULONG       bytesReturned;
    tPlkLedMem  plkLedMem;

    plkLedInstance_l.oplkFileHandle = CreateFile(PLK_DEV_FILE,    // Name of the NT "device" to open
                              GENERIC_READ | GENERIC_WRITE,         // Access rights requested
                              FILE_SHARE_READ | FILE_SHARE_WRITE,   // Share access - NONE
                              NULL,                                 // Security attributes - not used!
                              OPEN_EXISTING,                        // Device must exist to open it.
                              FILE_ATTRIBUTE_NORMAL,                // Open for overlapped I/O
                              NULL);                                // Extended attributes - not used!

    if (plkLedInstance_l.oplkFileHandle == INVALID_HANDLE_VALUE)
    {
        errCode = GetLastError();
        DEBUG_LVL_ERROR_TRACE("%s() CreateFile failed with error 0x%x\n", __func__, errCode);
        return;
    }

    if (!DeviceIoControl(plkLedInstance_l.oplkFileHandle, PLK_CMD_GET_PLK_LED_BASE,
        0, 0,
        &plkLedMem, sizeof(tPlkLedMem),
        &bytesReturned, NULL))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error in DeviceIoControl : %d\n", __func__, GetLastError());
        return;
    }
    
    if (bytesReturned == 0 || plkLedMem.pBaseAddr == NULL)
        return;

    plkLedInstance_l.pPlkLedReg = (UINT8*) plkLedMem.pBaseAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown GPIO module

The function shuts down the GPIO module.
*/
//------------------------------------------------------------------------------
void plkled_exit(void)
{
    ULONG       bytesReturned;
    tPlkLedMem  plkLedMem;

    plkled_setStatusLed(FALSE);
    plkled_setErrorLed(FALSE);

    plkLedMem.pBaseAddr = plkLedInstance_l.pPlkLedReg;

    if (!DeviceIoControl(plkLedInstance_l.oplkFileHandle, PLK_CMD_FREE_PLK_LED_BASE,
        0, 0,
        &plkLedMem, sizeof(tPlkLedMem),
        &bytesReturned, NULL))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error in DeviceIoControl : %d\n", __func__, GetLastError());
    }

    plkLedInstance_l.pPlkLedReg = NULL;

    CloseHandle(plkLedInstance_l.oplkFileHandle);
}

//------------------------------------------------------------------------------
/**
\brief  Sets the status LED

The function sets the POWERLINK status LED.

\param  fOn_p               Determines the LED state
*/
//------------------------------------------------------------------------------
void plkled_setStatusLed(BOOL fOn_p)
{
    UINT32  reg;

    reg = PLKLED_READ_REG(PLKLED_OUTSET_OFFSET);
    reg |= PLKLED_STATUS_MASK;

    if (fOn_p != FALSE)
    {
        PLKLED_WRITE_REG(PLKLED_OUTSET_OFFSET, reg);
    }
    else
    {
        PLKLED_WRITE_REG(PLKLED_OUTCLEAR_OFFSET, reg);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Sets the error LED

The function sets the POWERLINK error LED.

\param  fOn_p               Determines the LED state
*/
//------------------------------------------------------------------------------
void plkled_setErrorLed(BOOL fOn_p)
{
    UINT32  reg;

    reg = PLKLED_READ_REG(PLKLED_OUTSET_OFFSET);
    reg |= PLKLED_ERROR_MASK;

    if (fOn_p != FALSE)
    {
        PLKLED_WRITE_REG(PLKLED_OUTSET_OFFSET, reg);
    }
    else
    {
        PLKLED_WRITE_REG(PLKLED_OUTCLEAR_OFFSET, reg);
    }
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}

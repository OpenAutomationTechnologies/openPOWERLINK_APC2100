/**
********************************************************************************
\file   flash-nios2.c

\brief  Nios II Flash driver

This file implements handling the Nios II flash driver.
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
#include <flash.h>
#include <oplk/oplk.h>

#include <sys/alt_flash.h>
#include <system.h>

// Check if system.h provides the EPCS_FLASH_CONTROLLER parameters. If so the
// BSP provides the necessary headers.
// Otherwise this module degenerates to a null implementation.
#if defined(__ALTERA_AVALON_EPCS_FLASH_CONTROLLER)
#include <altera_avalon_epcs_flash_controller.h>

#define FLASH_NAME          EPCS_FLASH_CONTROLLER_NAME

#else
#define FLASH_NULL //TODO: Disable the functions, error if write/read?
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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
*  \brief Flash instance
*
*  The struct defines the Flash instance.
*/
typedef struct
{
    alt_flash_fd*   pFlashDevice;   ///< Altera Flash device instance
    tFlashInfo      flashInfo;      ///< Flash info
    BOOL            fInitialized;   ///< Flash module initialized

} tFlashInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tFlashInstance flashInstance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static int getFlashInfo(tFlashInfo* pFlashInfo_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize Flash module

The function initializes the Flash module before being used.

\return The function returns 0 if the Flash module has been initialized
        successfully, otherwise -1.
*/
//------------------------------------------------------------------------------
int flash_init(void)
{
    // Clear instance memory
    memset((void*)&flashInstance_g, 0, sizeof(tFlashInstance));

    flashInstance_g.pFlashDevice = alt_flash_open_dev(FLASH_NAME);
    if (flashInstance_g.pFlashDevice == NULL)
        return -1;

    if (getFlashInfo(&(flashInstance_g.flashInfo)) != 0)
        return -1;

    flashInstance_g.fInitialized = TRUE;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Exit Flash module

The function exits the Flash module.
*/
//------------------------------------------------------------------------------
void flash_exit(void)
{
    // Reset the initialized flag
    flashInstance_g.fInitialized = FALSE;

    alt_flash_close_dev(flashInstance_g.pFlashDevice);
}

//------------------------------------------------------------------------------
/**
\brief  Get Flash information

The function gets the information of the detected Flash.

\param  pFlashInfo_p    Pointer to Flash info structure which is set with
                        the required information.

\return The function returns 0 if the Flash info has been provided successfully,
        otherwise -1.
*/
//------------------------------------------------------------------------------
int flash_getInfo(tFlashInfo* pFlashInfo_p)
{
    if ((!flashInstance_g.fInitialized) || (pFlashInfo_p == NULL))
        return -1;

    *pFlashInfo_p = flashInstance_g.flashInfo;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Read from Flash

The function reads from the given Flash offset. The caller must provide a buffer
with sufficient size.

\param  offset_p    Base Flash offset reading from
\param  pDest_p     Pointer to destination buffer storing the read data
\param  length_p    Length of the data to be read from Flash

\return The function returns 0 if the Flash read operation was successful,
        otherwise -1.
*/
//------------------------------------------------------------------------------
int flash_read(UINT offset_p, UINT8* pDest_p, UINT length_p)
{
    int ret;

    if ((!flashInstance_g.fInitialized) ||
       (offset_p > flashInstance_g.flashInfo.size) ||
       ((offset_p + length_p) > flashInstance_g.flashInfo.size))
    {
        return -1;
    }

    ret = alt_read_flash(flashInstance_g.pFlashDevice, offset_p, pDest_p, length_p);

    // EPCS Flash read returns 0 on success.
    return (ret == 0) ? 0 : -1;
}

//------------------------------------------------------------------------------
/**
\brief  Erase Flash sector

The function erases the given Flash sector. Use flash_getInfo function to obtain
the Flash sector size and count.

\param  offset_p    Byte offset of sector

\return The function returns 0 if the sector erase operation was successful,
        otherwise -1.
*/
//------------------------------------------------------------------------------
int flash_eraseSector(UINT offset_p)
{
    int     ret;

    if ((!flashInstance_g.fInitialized) || (offset_p > flashInstance_g.flashInfo.size))
        return -1;

    ret = alt_erase_flash_block(flashInstance_g.pFlashDevice, offset_p,
                                flashInstance_g.flashInfo.sectorSize);

    // EPCS Flash erase returns 0 or positive value on success.
    return (ret >= 0) ? 0 : -1;
}

//------------------------------------------------------------------------------
/**
\brief  Write to Flash

The function writes to the given Flash offset.

\note Before writing to a sector that already holds content it is mandatory to
      backup that data and add it to newly written data.
      Otherwise the stored content will get lost due to a sector erase!

\param  offset_p    Base Flash offset writing to
\param  pSrc_p      Pointer to source buffer holding the data to be written
\param  length_p    Length of the data to be written to Flash

\return The function returns 0 if the write operation was successful,
        otherwise -1.
*/
//------------------------------------------------------------------------------
int flash_write(UINT offset_p, UINT8* pSrc_p, UINT length_p)
{
    int ret;
    int blockOffset;

    if ((!flashInstance_g.fInitialized) ||
        (offset_p > flashInstance_g.flashInfo.size) ||
        ((offset_p + length_p) > flashInstance_g.flashInfo.size))
    {
        return -1;
    }

    // Get the addressed block's byte address
    blockOffset = (offset_p / flashInstance_g.flashInfo.sectorSize) *
                  flashInstance_g.flashInfo.numberOfSectors;

    ret = alt_write_flash_block(flashInstance_g.pFlashDevice, 0, offset_p, pSrc_p, length_p);

    // EPCS Flash write returns 0 or positive value on success.
    return (ret >= 0) ? 0 : -1;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Get Flash information

The function gets the information of the detected Flash.

\param  pFlashInfo_p    Pointer to Flash info structure which is filled with
                        the required information.

\return The function returns 0 if the Flash info has been provided successfully,
        otherwise -1.
*/
//------------------------------------------------------------------------------
static int getFlashInfo(tFlashInfo* pFlashInfo_p)
{
    int             ret;
    int             numberOfRegions;
    flash_region*   pRegionInfo;

    // Get flash info, only support one region!
    ret = alt_get_flash_info(flashInstance_g.pFlashDevice, &pRegionInfo,
                             &numberOfRegions);
    if ((ret != 0) || (numberOfRegions != 1))
        return -1;

    pFlashInfo_p->size = pRegionInfo->region_size;
    pFlashInfo_p->numberOfSectors = pRegionInfo->number_of_blocks;
    pFlashInfo_p->sectorSize = pRegionInfo->block_size;

    return 0;
}

/// \}

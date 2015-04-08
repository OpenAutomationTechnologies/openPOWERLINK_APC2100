/**
********************************************************************************
\file   firmware.h

\brief  Firmware driver

Firmware driver header file.

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

#ifndef _INC_firmware_H_
#define _INC_firmware_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>

#if defined(__NIOS2__)
#include "firmware-nios2.h"
#else
#error "Target not supported!"
#endif

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define FIRMWARE_INVALID_IMAGE_BASE     0xFFFFFFFF

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
*  \brief Firmware image enum
*
*  This enum is used to identify a Firmware image type.
*/
typedef enum
{
    kFirmwareImageUnknown       = 0,    ///< Unknown image
    kFirmwareImageFactory       = 1,    ///< Factory image
    kFirmwareImageUpdate        = 2,    ///< Update image

} eFirmwareImageType;

typedef UINT32 tFirmwareImageType;

/**
*  \brief Firmware status enum
*
*  This enum is used to identify the Firmware module status.
*/
typedef enum
{
    kFirmwareStatusUnknown      = 0,    ///< Firmware status unknown
    kFirmwareStatusPor          = 1,    ///< Firmware comes from power-on-reset
    kFirmwareStatusReconfig     = 2,    ///< Firmware comes from reconfiguration
    kFirmwareStatusError        = 3,    ///< Firmware comes from error

} eFirmwareStatus;

typedef UINT32 tFirmwareStatus;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

int                 firmware_init(void);
void                firmware_exit(void);

tFirmwareImageType  firmware_getCurrentImageType(void);
tFirmwareStatus     firmware_getStatus(void);

UINT32              firmware_getImageBase(tFirmwareImageType sel_p);
UINT32              firmware_getDeviceHeaderBase(void);
int                 firmware_calcCrc(UINT32* pCrcVal_p, UINT8* pBuffer_p, INT length_p);
int                 firmware_checkHeader(tFirmwareHeader* pHeader_p);
int                 firmware_checkDeviceHeader(tFirmwareDeviceHeader* pHeader_p);

void                firmware_process(void);
void                firmware_reconfig(tFirmwareImageType next_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_firmware_H_ */

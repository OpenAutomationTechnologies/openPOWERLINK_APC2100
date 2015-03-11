/**
********************************************************************************
\file   firmware-nios2.h

\brief  Firmware driver for Nios II

Firmware driver header file for Nios II.

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

#ifndef _INC_firmware_nios2_H_
#define _INC_firmware_nios2_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define FIRMWARE_HEADER_SIGNATUR    0x46575550  ///< Header signature
#define FIRMWARE_HEADER_VERSION     0x00000001  ///< Header version

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
*  \brief Firmware update image header
*
*  The struct defines the firmware header which is stored in configuration flash
*  before the firmware update image.
*/
typedef struct
{
    UINT32              signature;      ///< Firmware header signature
    UINT32              version;        ///< Firmware header version
    UINT32              timeStamp;      ///< Firmware image time stamp
    UINT32              length;         ///< Firmware image length
    UINT32              crc;            ///< Firmware image crc
    UINT32              oplkVersion;    ///< openPOWERLINK version
    UINT32              oplkFeature;    ///< openPOWERLINK feature
    UINT32              headerCrc;      ///< Firmware header crc
} tFirmwareHeader;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_firmware_nios2_H_ */

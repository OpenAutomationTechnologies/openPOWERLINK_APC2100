/**
********************************************************************************
\file   prodtestint.h

\brief  Post production test internal header

This file contains the definitions for the post production test.

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

#ifndef _INC_prodtestint_H_
#define _INC_prodtestint_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define PRODTEST_COMMAND_DATASIZE       256

#define PRODTEST_ETHERTYPE_ARP          0x0806
#define PRODTEST_ETHERTYPE_IP           0x0800

#define PRODTEST_ARP_HWTYPE             1
#define PRODTEST_ARP_PROTYPE            PRODTEST_ETHERTYPE_IP
#define PRODTEST_ARP_OPREQ              1
#define PRODTEST_ARP_OPRES              2

#define PRODTEST_IP_VHL                 0x45
#define PRODTEST_IP_PROTUDP             17
#define PRODTEST_UDP_PORT               3819
#define PRODTEST_UDP_MSGTYPE            6
#define PRODTEST_UDP_SVID               176

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

typedef struct
{
    UINT8               aDstMac[6];
    UINT8               aSrcMac[6];
    UINT16              etherType;
} tProdtestEthHdr;

typedef struct
{
    UINT8               vhl;
    UINT8               tos;
    UINT16              len;
    UINT16              ipId;
    UINT16              ipOffset;
    UINT8               ttl;
    UINT8               proto;
    UINT16              chksum;
    UINT8               aSrcIp[4];
    UINT8               aDstIp[4];
} tProdtestIpHdr;

typedef struct
{
    UINT16              srcPort;
    UINT16              dstPort;
    UINT16              len;
    UINT16              chksum;
    UINT8               messageType;
    UINT8               reserve[2];
    UINT8               serviceId;
} tProdtestUdpHdr;

typedef struct
{
    UINT8               aMessageId[4];
    UINT16              command;
    UINT16              error;
} tProdtestCmdHdr;

typedef struct
{
    tProdtestEthHdr     ethHeader;
    tProdtestIpHdr      ipHeader;
    tProdtestUdpHdr     udpHeader;
    tProdtestCmdHdr     pmeHeader;
    UINT8               data[PRODTEST_COMMAND_DATASIZE];
} tProdtestCmd;

typedef struct
{
    tProdtestEthHdr     ethHeader;
    UINT16              hardwareType;
    UINT16              protocolType;
    UINT8               hardwareAddressLength;
    UINT8               protocolAddressLength;
    UINT16              operation;
    UINT8               aSenderHardwareAddress[6];
    UINT8               aSenderProtocolAddress[4];
    UINT8               aTargetHardwareAddress[6];
    UINT8               aTargetProtocolAddress[4];
} tProdtestArp;

typedef enum
{
    kProdtestCommandNoTest          = 0,    ///< No production test
    kProdtestCommandCommunication   = 1,    ///< Communication test
    kProdtestCommandRam             = 6,    ///< RAM test
    kProdtestCommandSetMacAddress   = 15,   ///< Set MAC address to NV memory

} tProdtestCommand;
/* communication */
//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_prodtestint_H_ */

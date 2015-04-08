/**
********************************************************************************
\file   prodtest.c

\brief  Post Production Test for Nios II

This file implements the Nios II specific parts for the Post Production Test
functionality.

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
#include "prodtest.h"
#include "prodtestint.h"

#include <common/oplkinc.h>
#include <common/target.h>
#include <kernel/edrv.h>
#include <kernel/dllkfilter.h>

#include <flash.h>
#include <firmware.h>

#ifdef __NIOS2__
#include <io.h>
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
#if CHECK_IF_BIG_ENDIAN()
// Big endian => no swap needed
#define htons(x)        (x)
#define htonl(x)        (x)
#define ntohs(x)        (x)
#define ntohl(x)        (x)
#else
// Little endian => swap needed
// Swap long: 0x00C0FFEE --> 0xEEFFC000
#define ARP_SWAPL(x)    ((((x) >> 24) & 0x000000FF) | (((x) >> 8) & 0x0000FF00) | \
                        (((x) & 0x000000FF) << 24) | (((x) & 0x0000FF00) << 8))

// Swap short: 0xC0FE --> 0xFEC0
#define ARP_SWAPS(x)    ((((x) >> 8) & 0x00FF) | (((x) << 8) & 0xFF00))

#define htons(x)        ARP_SWAPS(x)
#define htonl(x)        ARP_SWAPL(x)
#define ntohs(x)        ARP_SWAPS(x)
#define ntohl(x)        ARP_SWAPL(x)
#endif //CHECK_IF_BIG_ENDIAN

#ifdef __NIOS2__
#define IO_WR32(addr, val)      IOWR_32DIRECT(addr, 0, val)
#define IO_WR16(addr, val)      IOWR_16DIRECT(addr, 0, val)
#define IO_WR8(addr, val)       IOWR_8DIRECT(addr, 0, val)
#define IO_RD32(addr)           (UINT32)IORD_32DIRECT(addr, 0)
#define IO_RD16(addr)           (UINT16)IORD_16DIRECT(addr, 0)
#define IO_RD8(addr)            (UINT8)IORD_8DIRECT(addr, 0)
#else
#define IO_WR32(addr, val)      *((volatile UINT32*)addr) = val
#define IO_WR16(addr, val)      *((volatile UINT16*)addr) = val
#define IO_WR8(addr, val)       *((volatile UINT8*)addr) = val
#define IO_RD32(addr)           *((volatile UINT32*)addr)
#define IO_RD16(addr)           *((volatile UINT16*)addr)
#define IO_RD8(addr)            *((volatile UINT8*)addr)
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
 * \brief Post production test instance
 *
 * This struct defines the post production test instance.
 *
 */
typedef struct
{
    BOOL            fInitialize;        ///< Instance is initialized
    UINT8           aMacAddress[6];     ///< Local MAC address
    UINT8           aIpAddress[4];      ///< Local IP address
    tEdrvTxBuffer   txBufArpResponse;   ///< Tx buffer descriptor for ARP response
    tEdrvTxBuffer   aTxBufCmdReply[2];  ///< Tx buffer descriptor array for CMD reply
    UINT8*          pMemTestBuffer;     ///< Memory for memory tests

} tProductiontest;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tProductiontest prodtestInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tEdrvReleaseRxBuffer edrvRxCb(tEdrvRxBuffer* pRxBuffer_p);
static void edrvTxCb(tEdrvTxBuffer* pTxBuffer_p);
static int handleRxArpFrame(tPlkFrame* pFrame_p, UINT size_p);
static int handleRxProdtestFrame(tPlkFrame* pFrame_p, UINT size_p);
static UINT16 calcIpHdrChecksum(tProdtestIpHdr* pIpHdr_p);
static int initArpResp(tEdrvTxBuffer* pTxBuffer_p, int bufCnt_p);
static int initCmdReply(tEdrvTxBuffer* pTxBuffer_p, int bufCnt_p);
static int memoryTest(UINT8* pBase_p, int length_p);
static int writeMacAddress(UINT8* pMacAddr_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize post production test module

The function initializes the post production test module before being used.

\return The function returns 0 if initialization was successful, otherwise -1
*/
//------------------------------------------------------------------------------
int prodtest_init(void)
{
    tEdrvInitParam  edrvInit;
    tEdrvFilter     aEdrvFilter[2];
    UINT            edrvFilterChange;
    UINT8           aMacAddr[] = {POSTPROTEST_MACADDR};
    UINT8           aIpAddr[] = {POSTPROTEST_IPADDR};

    OPLK_MEMSET((void*)&prodtestInstance_l, 0, sizeof(tProductiontest));
    OPLK_MEMSET((void*)&edrvInit, 0, sizeof(tEdrvInitParam));
    OPLK_MEMSET((void*)&aEdrvFilter, 0, sizeof(tEdrvFilter));

    OPLK_MEMCPY((void*)prodtestInstance_l.aMacAddress, aMacAddr, 6);
    OPLK_MEMCPY((void*)prodtestInstance_l.aIpAddress, aIpAddr, 4);

    OPLK_MEMCPY((void*)edrvInit.aMacAddr, (void*)aMacAddr, 6);
    edrvInit.pfnRxHandler = edrvRxCb;

    if (edrv_init(&edrvInit) != kErrorOk)
        return -1;

    OPLK_MEMSET(aEdrvFilter, 0, sizeof(aEdrvFilter));

    // Set first Rx filter to receive unicasts
    aEdrvFilter[0].handle = 0;
    aEdrvFilter[0].fEnable = TRUE;
    aEdrvFilter[0].pTxBuffer = NULL;
    aEdrvFilter[0].aFilterMask[0] = 0xFF;
    aEdrvFilter[0].aFilterMask[1] = 0xFF;
    aEdrvFilter[0].aFilterMask[2] = 0xFF;
    aEdrvFilter[0].aFilterMask[3] = 0xFF;
    aEdrvFilter[0].aFilterMask[4] = 0xFF;
    aEdrvFilter[0].aFilterMask[5] = 0xFF;
    aEdrvFilter[0].aFilterValue[0] = prodtestInstance_l.aMacAddress[0];
    aEdrvFilter[0].aFilterValue[1] = prodtestInstance_l.aMacAddress[1];
    aEdrvFilter[0].aFilterValue[2] = prodtestInstance_l.aMacAddress[2];
    aEdrvFilter[0].aFilterValue[3] = prodtestInstance_l.aMacAddress[3];
    aEdrvFilter[0].aFilterValue[4] = prodtestInstance_l.aMacAddress[4];
    aEdrvFilter[0].aFilterValue[5] = prodtestInstance_l.aMacAddress[5];

    // Set second Rx filter to receive broadcasts
    aEdrvFilter[1].handle = 1;
    aEdrvFilter[1].fEnable = TRUE;
    aEdrvFilter[1].pTxBuffer = NULL;
    aEdrvFilter[1].aFilterMask[0] = 0xFF;
    aEdrvFilter[1].aFilterMask[1] = 0xFF;
    aEdrvFilter[1].aFilterMask[2] = 0xFF;
    aEdrvFilter[1].aFilterMask[3] = 0xFF;
    aEdrvFilter[1].aFilterMask[4] = 0xFF;
    aEdrvFilter[1].aFilterMask[5] = 0xFF;
    aEdrvFilter[1].aFilterValue[0] = 0xFF;
    aEdrvFilter[1].aFilterValue[1] = 0xFF;
    aEdrvFilter[1].aFilterValue[2] = 0xFF;
    aEdrvFilter[1].aFilterValue[3] = 0xFF;
    aEdrvFilter[1].aFilterValue[4] = 0xFF;
    aEdrvFilter[1].aFilterValue[5] = 0xFF;

    edrvFilterChange = UINT_MAX; //jz

    if (edrv_changeRxFilter(aEdrvFilter, 2, 2, edrvFilterChange) != kErrorOk)
        return -1;

    if (initArpResp(&prodtestInstance_l.txBufArpResponse, 1) != 0)
        return -1;

    if (initCmdReply(prodtestInstance_l.aTxBufCmdReply, tabentries(prodtestInstance_l.aTxBufCmdReply)) != 0)
        return -1;

    prodtestInstance_l.pMemTestBuffer = (UINT8*)malloc(POSTPROTEST_MEMTEST_SIZE);
    if (prodtestInstance_l.pMemTestBuffer == NULL)
        return -1;

    prodtestInstance_l.fInitialize = TRUE;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown post production test module

The function shuts down the post production test module.

*/
//------------------------------------------------------------------------------
void prodtest_exit(void)
{
    UINT i;

    if (!prodtestInstance_l.fInitialize)
        return;

    // Enter critical section since it could be that some packet is received,
    // which leads to accessing freed memory/buffers/...
    target_enableGlobalInterrupt(FALSE);

    prodtestInstance_l.fInitialize = FALSE;

    free(prodtestInstance_l.pMemTestBuffer);
    prodtestInstance_l.pMemTestBuffer = NULL;

    for (i=0; i<tabentries(prodtestInstance_l.aTxBufCmdReply); i++)
        edrv_freeTxBuffer(&prodtestInstance_l.aTxBufCmdReply[i]);

    edrv_freeTxBuffer(&prodtestInstance_l.txBufArpResponse);

    edrv_shutdown();

    target_enableGlobalInterrupt(TRUE);
}

//------------------------------------------------------------------------------
/**
\brief  Post production test process function

This is the post production test process function. It shall be called on a
regular basis.

\return The function returns 0 if initialization was successful, otherwise -1
*/
//------------------------------------------------------------------------------
int prodtest_process(void)
{
    tOplkError      ret;
    UINT            index;
    tEdrvTxBuffer*  apTxBuffer[] =
    {
        &prodtestInstance_l.txBufArpResponse,
        &prodtestInstance_l.aTxBufCmdReply[0],
        &prodtestInstance_l.aTxBufCmdReply[1]
    };

    if (!prodtestInstance_l.fInitialize)
        return 0; // silent ignore

    for (index = 0; index < tabentries(apTxBuffer); index++)
    {
        if (apTxBuffer[index]->txFrameSize > 1)
        {
            ret = edrv_sendTxBuffer(apTxBuffer[index]);
            if (ret != kErrorOk)
                return -1;

            apTxBuffer[index]->txFrameSize = 1;
        }
    }

    return 0;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Frame Rx callback

This is the Rx callback function called by the Edrv when a frame is received.

\param  pRxBuffer_p     Rx buffer descriptor for the received frame.

\return The function returns a tOplkError code.
*/
//------------------------------------------------------------------------------
static tEdrvReleaseRxBuffer edrvRxCb(tEdrvRxBuffer* pRxBuffer_p)
{
    int         ret;
    tPlkFrame*  pFrame;
    UINT        frameSize;

    if (!prodtestInstance_l.fInitialize)
        goto Exit;

    if (pRxBuffer_p == NULL)
        goto Exit; // silently drop

    if (pRxBuffer_p->pBuffer == NULL)
        goto Exit; // silently drop

    pFrame = (tPlkFrame*)pRxBuffer_p->pBuffer;
    frameSize = pRxBuffer_p->rxFrameSize;

    PRINTF("Received frame with EtherType 0x%04X\n", ntohs(pFrame->etherType));

    if ((ret = handleRxArpFrame(pFrame, frameSize)) == 0)
        goto Exit;

    if ((ret = handleRxProdtestFrame(pFrame, frameSize)) == 0)
        goto Exit;

    /* Any other frame can be handled here... */

    PRINTF(" -> not handled, dropped!\n");

Exit:
    return kEdrvReleaseRxBufferImmediately;
}

//------------------------------------------------------------------------------
/**
\brief  Frame Tx callback

This is the Tx callback function called by the Edrv when a frame is transmitted.

\param  pTxBuffer_p     Tx buffer descriptor for the transmitted frame.

\return The function returns a tOplkError code.
*/
//------------------------------------------------------------------------------
static void edrvTxCb(tEdrvTxBuffer* pTxBuffer_p)
{
    if (!prodtestInstance_l.fInitialize)
        return;

    // Mark frame as being send...
    pTxBuffer_p->txFrameSize = 0;
}

//------------------------------------------------------------------------------
/**
\brief  Handle ARP frame

This is function processes the given frame if it is an ARP frame meant for the
local node.

\param  pFrame_p    Pointer to Ethernet frame received.
\param  size_p      Size of the frame.

\return The function returns if the frame was processed.
\retval 0           The frame was processed successfully.
\retval -1          The frame was not processed, try with other protocol.
*/
//------------------------------------------------------------------------------
static int handleRxArpFrame(tPlkFrame* pFrame_p, UINT size_p)
{
    tProdtestArp*   pArpReq = (tProdtestArp*)pFrame_p;

    UNUSED_PARAMETER(size_p);

    // Check if it is an ARP request
    if ((ntohs(pArpReq->ethHeader.etherType) == PRODTEST_ETHERTYPE_ARP) &&
        (ntohs(pArpReq->hardwareType) == PRODTEST_ARP_HWTYPE) &&
        (ntohs(pArpReq->protocolType) == PRODTEST_ETHERTYPE_IP) &&
        (pArpReq->hardwareAddressLength == 6) &&
        (pArpReq->protocolAddressLength == 4) &&
        (ntohs(pArpReq->operation) == PRODTEST_ARP_OPREQ) &&
        (OPLK_MEMCMP(pArpReq->aTargetProtocolAddress, prodtestInstance_l.aIpAddress, 4) == 0))
    {
        // This is an ARP request meant for us, send reply!

        // Check if reply Tx buffer is free
        if (prodtestInstance_l.txBufArpResponse.txFrameSize == 0)
        {
            tEdrvTxBuffer*  pTxBuffer = &prodtestInstance_l.txBufArpResponse;
            tProdtestArp*   pArpRes = (tProdtestArp*)pTxBuffer->pBuffer;

            pTxBuffer->txFrameSize = 1; // Fill in progress

            // Set destination MAC => copy source MAC from Rx frame
            OPLK_MEMCPY(pArpRes->ethHeader.aDstMac, pArpReq->ethHeader.aSrcMac, 6);

            // Set sender IP Address / MAC
            OPLK_MEMCPY(pArpRes->aSenderProtocolAddress, prodtestInstance_l.aIpAddress, 4);
            OPLK_MEMCPY(pArpRes->aSenderHardwareAddress, prodtestInstance_l.aMacAddress, 6);

            // Copy received sender info to target info
            OPLK_MEMCPY(pArpRes->aTargetHardwareAddress, pArpReq->aSenderHardwareAddress, 6);
            OPLK_MEMCPY(pArpRes->aTargetProtocolAddress, pArpReq->aSenderProtocolAddress, 4);

            pTxBuffer->txFrameSize = sizeof(tProdtestArp); // Ready for Tx
        }
    }
    else
    {
        // This is not an ARP request to us, drop it
        return -1;
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Handle production test command frame

This is function processes the given frame. If it is a production test command
frame, it handles it accordingly.

\param  pFrame_p    Pointer to Ethernet frame received.
\param  size_p      Size of the frame.

\return The function returns if the frame was processed.
\retval 0           The frame was processed successfully.
\retval -1          The frame was not processed, try with other protocol.
*/
//------------------------------------------------------------------------------
static int handleRxProdtestFrame(tPlkFrame* pFrame_p, UINT size_p)
{
    tProdtestCmd*   pCmd = (tProdtestCmd*)pFrame_p;
    UINT            i;

    UNUSED_PARAMETER(size_p);

    // Check for production test frame to us
    if ((ntohs(pCmd->ethHeader.etherType) == PRODTEST_ETHERTYPE_IP) &&
        (pCmd->ipHeader.vhl == PRODTEST_IP_VHL) &&
        (pCmd->ipHeader.proto == PRODTEST_IP_PROTUDP) &&
        (OPLK_MEMCMP(pCmd->ipHeader.aDstIp, prodtestInstance_l.aIpAddress, 4) == 0) &&
        (ntohs(pCmd->udpHeader.dstPort) == PRODTEST_UDP_PORT) &&
        (pCmd->udpHeader.messageType == PRODTEST_UDP_MSGTYPE) &&
        (pCmd->udpHeader.serviceId == PRODTEST_UDP_SVID))
    {
        // This is a production test command frame
        PRINTF("Received PRODUCTION TEST COMMAND FRAME!\n");

        for (i=0; i<tabentries(prodtestInstance_l.aTxBufCmdReply); i++)
        {
            // Check if reply Tx buffer is free
            if (prodtestInstance_l.aTxBufCmdReply[i].txFrameSize == 0)
            {
                tEdrvTxBuffer*  pTxBuffer = &prodtestInstance_l.aTxBufCmdReply[i];
                tProdtestCmd*   pResp = (tProdtestCmd*)pTxBuffer->pBuffer;

                pTxBuffer->txFrameSize = 1; // Fill in progress

                OPLK_MEMSET(pResp->data, 0, sizeof(pResp->data));

                OPLK_MEMCPY(pResp->ethHeader.aDstMac, pCmd->ethHeader.aSrcMac, 6);
                OPLK_MEMCPY(pResp->pmeHeader.aMessageId, pCmd->pmeHeader.aMessageId, sizeof(pResp->pmeHeader.aMessageId));

                pResp->udpHeader.dstPort = pCmd->udpHeader.srcPort;
                pResp->pmeHeader.command = pCmd->pmeHeader.command;
                pResp->pmeHeader.error = 0;

                OPLK_MEMCPY(pResp->ipHeader.aDstIp, pCmd->ipHeader.aSrcIp, 4);

                pResp->ipHeader.chksum = 0;
                pResp->ipHeader.chksum = calcIpHdrChecksum(&pResp->ipHeader);

                switch (pCmd->pmeHeader.command)
                {
                    case kProdtestCommandCommunication:
                        // Nothing special to do, just send the response frame
                        PRINTF(" --> kProdtestCommandCommunication\n");
                        break;

                    case kProdtestCommandRam:
                        PRINTF(" --> kProdtestCommandRam\n");

                        pResp->pmeHeader.error = memoryTest(prodtestInstance_l.pMemTestBuffer,
                                                            POSTPROTEST_MEMTEST_SIZE);

                        break;

                    case kProdtestCommandSetMacAddress:
                        PRINTF(" --> kProdtestCommandSetMacAddress\n");
                        pResp->pmeHeader.error = writeMacAddress(pCmd->data);

                        if (pResp->pmeHeader.error == 0)
                        {
                            tFirmwareDeviceHeader   deviceHdr;
                            UINT32                  offset = firmware_getDeviceHeaderBase();

                            flash_read(offset, (UINT8*)&deviceHdr, sizeof(tFirmwareDeviceHeader));

                            // Ignore invalid device header, simply return whatever is read

                            OPLK_MEMCPY(pResp->data, deviceHdr.aMacAddr, 6);
                        }

                        break;

                    default:
                        // Unknown test
                        pResp->pmeHeader.error = 1;
                        break;
                }

                pTxBuffer->txFrameSize = sizeof(tProdtestCmd); // Ready for Tx

                break;
            }
        } // for tabentries(prodtestInstance_l.aTxBufCmdReply)
    }
    else
    {
        // This is no production test command frame, drop it
        return -1;
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Calculate IP header checksum

This is function calculates the IP header checksum of the given IP header.

\param  pIpHdr_p    Pointer to IP header

\return The function returns the calculated checksum.
*/
//------------------------------------------------------------------------------
static UINT16 calcIpHdrChecksum(tProdtestIpHdr* pIpHdr_p)
{
    UINT    length;
    UINT16* pFrame;
    UINT32  sum = 0;
    UINT16  carry;
    UINT16  result;

    // IP header length is given in words
    length = (pIpHdr_p->vhl & 0xF) * 2;

    // Set 16 bit pointer to IP header
    pFrame = (UINT16*)pIpHdr_p;

    // Get 32 bit sum of IP header
    while (length)
    {
        sum += *pFrame;
        pFrame++;
        length--;
    }

    // Add upper word of 32 bit result to lower word
    do
    {
        result = sum & 0x0000FFFF;
        carry = (sum & 0xFFFF0000) >> 16;

        sum = result + carry;
    } while (carry);

    return ~result;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize ARP response Tx buffer

This is function initializes the ARP response Tx buffer.

\param  pTxBuffer_p Pointer to Tx buffer descriptor (first entry)
\param  bufCnt_p    Number of Tx buffer descriptors

\return The function returns 0 on success, -1 otherwise.
*/
//------------------------------------------------------------------------------
static int initArpResp(tEdrvTxBuffer* pTxBuffer_p, int bufCnt_p)
{
    tProdtestArp* pFrame;

    PRINTF("%s(0x%08X, %d)\n", __func__, (UINT)pTxBuffer_p, bufCnt_p);

    for (; bufCnt_p; bufCnt_p--, pTxBuffer_p++)
    {
        OPLK_MEMSET(pTxBuffer_p, 0, sizeof(tEdrvTxBuffer));

        pTxBuffer_p->maxBufferSize = sizeof(tProdtestArp);

        if (edrv_allocTxBuffer(pTxBuffer_p) != kErrorOk)
            return -1;

        pFrame = (tProdtestArp*)pTxBuffer_p->pBuffer;

        // Prepare ARP response frame
        OPLK_MEMCPY(pFrame->ethHeader.aSrcMac, prodtestInstance_l.aMacAddress, 6);
        pFrame->ethHeader.etherType = htons(PRODTEST_ETHERTYPE_ARP);
        pFrame->hardwareType = htons(PRODTEST_ARP_HWTYPE);
        pFrame->protocolType = htons(PRODTEST_ARP_PROTYPE);
        pFrame->hardwareAddressLength = 6;
        pFrame->protocolAddressLength = 4;
        pFrame->operation = htons(PRODTEST_ARP_OPRES);

        pTxBuffer_p->pfnTxHandler = edrvTxCb;
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize Commnad reply Tx buffer

This is function initializes the command reply Tx buffer.

\param  pTxBuffer_p Pointer to Tx buffer descriptor (first entry)
\param  bufCnt_p    Number of Tx buffer descriptors

\return The function returns 0 on success, -1 otherwise.
*/
//------------------------------------------------------------------------------
static int initCmdReply(tEdrvTxBuffer* pTxBuffer_p, int bufCnt_p)
{
    tProdtestCmd* pFrame;

    PRINTF("%s(0x%08X, %d)\n", __func__, (UINT)pTxBuffer_p, bufCnt_p);

    for (; bufCnt_p; bufCnt_p--, pTxBuffer_p++)
    {
        OPLK_MEMSET(pTxBuffer_p, 0, sizeof(tEdrvTxBuffer));

        pTxBuffer_p->maxBufferSize = sizeof(tProdtestCmd);

        if (edrv_allocTxBuffer(pTxBuffer_p) != kErrorOk)
            return -1;

        pFrame = (tProdtestCmd*)pTxBuffer_p->pBuffer;

        // Prepare command reply
        OPLK_MEMCPY(pFrame->ethHeader.aSrcMac, prodtestInstance_l.aMacAddress, 6);
        pFrame->ethHeader.etherType = htons(PRODTEST_ETHERTYPE_IP);
        pFrame->ipHeader.vhl = PRODTEST_IP_VHL;
        pFrame->ipHeader.tos = 0x00;
        pFrame->ipHeader.len = htons(sizeof(tProdtestCmd) - sizeof(tProdtestEthHdr));
        pFrame->ipHeader.ipId = htons(0x000);
        pFrame->ipHeader.ipOffset = htons(0x0000);
        pFrame->ipHeader.ttl = 0xFF;
        pFrame->ipHeader.proto = PRODTEST_IP_PROTUDP;
        OPLK_MEMCPY(pFrame->ipHeader.aSrcIp, prodtestInstance_l.aIpAddress, 4);
        OPLK_MEMSET(pFrame->ipHeader.aDstIp, 0xFF, 4);
        pFrame->udpHeader.srcPort = htons(PRODTEST_UDP_PORT);
        pFrame->udpHeader.len = htons(ntohs(pFrame->ipHeader.len) - sizeof(tProdtestIpHdr));
        pFrame->udpHeader.chksum = htons(0x0000); // Don't use this checksum
        pFrame->udpHeader.messageType = PRODTEST_UDP_MSGTYPE;
        pFrame->udpHeader.reserve[0] = 0x00;
        pFrame->udpHeader.reserve[1] = 0x00;
        pFrame->udpHeader.serviceId = PRODTEST_UDP_SVID;

        pTxBuffer_p->pfnTxHandler = edrvTxCb;
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Run memory test

This function runs a memory test for the given memory.

\param  pBase_p     Base address of memory to be tested
\param  length_p    Length of the memory to be tested in bytes

\return The function returns 0 on success, 1 otherwise.
*/
//------------------------------------------------------------------------------
static int memoryTest(UINT8* pBase_p, int length_p)
{
    int     offset;
    UINT32  testVal;

    for (offset=0; offset<length_p; offset++)
        IO_WR8(pBase_p + offset, offset);

    for (offset=0; offset<length_p; offset++)
    {
        if ((UINT8)offset != IO_RD8(pBase_p + offset))
            return 1;
    }

    for (offset=0; offset<length_p; offset+=2)
        IO_WR16(pBase_p + offset, offset);

    for (offset=0; offset<length_p; offset+=2)
    {
        if ((UINT16)offset != IO_RD16(pBase_p + offset))
            return 1;
    }

    for (offset=0; offset<length_p; offset+=4)
        IO_WR32(pBase_p + offset, offset);

    for (offset=0; offset<length_p; offset+=4)
    {
        if ((UINT32)offset != IO_RD32(pBase_p + offset))
            return 1;
    }

    testVal = 0xAAAAAAAA;
    for (offset=0; offset<length_p; offset+=4)
        IO_WR32(pBase_p + offset, testVal);

    for (offset=0; offset<length_p; offset+=4)
    {
        if (testVal != IO_RD32(pBase_p + offset))
            return 1;
    }

    testVal = 0x55555555;
    for (offset=0; offset<length_p; offset+=4)
        IO_WR32(pBase_p + offset, testVal);

    for (offset=0; offset<length_p; offset+=4)
    {
        if (testVal != IO_RD32(pBase_p + offset))
            return 1;
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Write MAC address to flash

This function writes the provided MAC address to flash.

\param  pMacAddr_p  Pointer to MAC address

\return The function returns 0 on success, 1 otherwise.
*/
//------------------------------------------------------------------------------
static int writeMacAddress(UINT8* pMacAddr_p)
{
    tFlashInfo              flashInfo;
    UINT8*                  pSectorBuffer;
    UINT32                  offset;
    UINT32                  sectorOffset;
    tFirmwareDeviceHeader*  pDeviceHeader;
    UINT32                  crcval = 0xFFFFFFFF;

    if (flash_getInfo(&flashInfo) != 0)
        return 1;

    offset = firmware_getDeviceHeaderBase();
    if (offset == FIRMWARE_INVALID_IMAGE_BASE)
        return 1;

    pSectorBuffer = OPLK_MALLOC(flashInfo.sectorSize);
    if (pSectorBuffer == NULL)
        return 1;

    // Get offset of the sector that includes the device header
    sectorOffset = offset / flashInfo.sectorSize;
    sectorOffset *= flashInfo.sectorSize;

    // Read out the sector
    if (flash_read(sectorOffset, pSectorBuffer, flashInfo.sectorSize) != 0)
        return 1;

    // Set device header pointer to device header offset
    pDeviceHeader = (tFirmwareDeviceHeader*)(pSectorBuffer + (offset - sectorOffset));

    //jz: Check if there is already a header, however, if there is, overwrite it!
    pDeviceHeader->signature = FIRMWARE_DEVICE_HEADER_SIGNATURE;
    pDeviceHeader->version = FIRMWARE_DEVICE_HEADER_VERSION;
    OPLK_MEMCPY(pDeviceHeader->aMacAddr, pMacAddr_p, 6);

    // Calculate header crc
    if (firmware_calcCrc(&crcval, (UINT8*)pDeviceHeader, sizeof(tFirmwareDeviceHeader) - 4) != 0)
        return 1;

    pDeviceHeader->headerCrc = crcval;

    // Erase sector (hope that no one cuts power!)
    if (flash_eraseSector(sectorOffset) != 0)
        return 1;

    // Write sector buffer to sector
    if (flash_write(sectorOffset, pSectorBuffer, flashInfo.sectorSize) != 0)
        return 1;

    if (pSectorBuffer != NULL)
        OPLK_FREE(pSectorBuffer);

    return 0;
}

/// \}

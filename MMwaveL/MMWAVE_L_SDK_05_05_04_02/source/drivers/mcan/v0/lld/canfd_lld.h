/*
 * Copyright (C) 2022 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/**
 *   \file  canfd_lld.h
 *
 *   \brief
 *      This is the header file for the CANFD driver which exposes the
 *      data structures and exported API which can be used by the
 *      applications to use the CANFD driver.
 */

/**
 *  \defgroup DRV_CANFD_MODULE CANFD Driver
 *
 *  The CANFD driver provides functionality of transferring data between CANFD peripherals.
 *  This driver does not interpret any of the data sent to or received from using this peripheral.
 *
 *  ## Initializing the driver #
 *  The CANFD Driver needs to be initialized once across the System. This is
 *  done using the #CANFD_lld_init. None of the CANFD API's can be used without invoking
 *  this API.
 *
 *  Once the CANFD Driver has been initialized; the bit timing can be configured using #CANFD_lld_configBitTime.
 *  This APIs can be called multiple times to reconfigure bit timings.
 *
 * ## Creating the message objects #
 *  Message objects are used to transmit or receive data over the CANFD peripheral. A message object is created
 *  using #CANFD_lld_createMsgObject.
 *
 * ## Sending and receiving data #
 *  Data is transmitted using the #CANFD_lld_write. The application will be notified when the transmit is
 *  complete if it has enabled dataInterruptEnable and registered a callback function appDataCallBack when
 *  initializing the CANFD driver.
 *
 *  If the receive interrupts are enabled using dataInterruptEnable field and a callback function appCallBack
 *  has been registered when initializing the CANFD driver, the driver notifies the application
 *  when the data has arrived. The application needs to call the #CANFD_lld_read function to read the received data.

 *
 * ## Error and status handling #
 *  The application can monitor the ECC error, Bus off error and Protocol Errors by enabling the error interrupts errInterruptEnable.
 *  The driver will call the registered callback function appErrCallBack to indicate which error fields caused the interrupt.
 *  It is up to the application to take appropriate action.
 *
 * ## Get/Set Options #
 *  Helper APIs to get and set various statistics, error counters, ECC diagnostics, power down have been provided.
 *  Refer to #CANFDLLD_Option for more information.
 *
 * ## Limitation #
 *  The CANFD driver does not support the DMA or power down.
 *
 *
 */

#ifndef CANFD_LLD_H_
#define CANFD_LLD_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stddef.h>
#include <drivers/mcan/v0/lld/mcan_lld.h>
#include <drivers/hw_include/cslr_soc.h>

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

#define ATTRIBUTE_WEAK      __attribute__((weak))

/*!
 *  \brief      Maximum number of Tx message objects that can be supported by CANFD
 */
#define MCAN_MAX_TX_MSG_OBJECTS             (32U)

/*!
 *  \brief      Maximum number of Rx message objects that can be supported by CANFD
 */
#define MCAN_MAX_RX_MSG_OBJECTS             (32U)

/*!
 *  \brief      Maximum number of message objects that can be supported by CANFD
 */
#define MCAN_MAX_MSG_OBJECTS                (MCAN_MAX_TX_MSG_OBJECTS + MCAN_MAX_RX_MSG_OBJECTS)

/*! \brief  Standard ID Filter Element Size */
#define MCAN_MSG_RAM_STD_ELEM_SIZE          (1U)

/*! \brief  Extended ID Filter Element Size */
#define MCAN_MSG_RAM_EXT_ELEM_SIZE          (2U)

/*! \brief  MCAN Header size in Bytes */
#define MCAN_MSG_HEADER_SIZE                (8U)

/*! \brief  Tx/Rx Element Size.
 * 18 words = 18 * 4 = 72 bytes: 8 bytes of header and 64 bytes of data */
#define MCAN_MSG_RAM_TX_RX_ELEM_SIZE        (18U)

/*! \brief  Message Identifier Masks */
#define XTD_MSGID_MASK                      (0x1fffffffU)
#define STD_MSGID_MASK                      (0x7ffU)
#define STD_MSGID_SHIFT                     (18U)

/*! \brief  Maximum payload supported by CAN-FD protocol in bytes. */
#define MCAN_MAX_PAYLOAD_BYTES              (64U)

/*! \brief  Maximum number of Rx buffers. */
#define MCAN_MAX_RX_BUFFERS                 (64U)

/*! \brief  Maximum number of Tx buffers. */
#define MCAN_MAX_TX_BUFFERS                 (32U)

/*! \brief Macro to get the size of an array */
#define CANFD_UTILS_ARRAYSIZE(x)  (sizeof(x) / sizeof(x[0]))

/*! \brief Get the index of the given element within an array */
#define CANFD_UTILS_GETARRAYINDEX(member, array)   (member - &array[0])

/*! \brief Macro to determine if a member is part of an array */
#define CANFD_UTILS_ARRAYISMEMBER(member, array)                              \
    (((((uint32)member - (uint32) & array[0]) % sizeof(array[0])) == 0)       \
     && (member >= &array[0])                                                 \
     && (CANFD_UTILS_GETARRAYINDEX(member, array) < CANFD_UTILS_ARRAYSIZE(array)))

/**
 * \brief  Defines all the interrupt are enabled.
 */
#define MCAN_INTR_MASK       ((uint32_t)MCAN_INTR_SRC_RX_FIFO0_NEW_MSG | \
                              (uint32_t)MCAN_INTR_SRC_RX_FIFO0_MSG_LOST |  \
                              (uint32_t)MCAN_INTR_SRC_RX_FIFO1_NEW_MSG | \
                              (uint32_t)MCAN_INTR_SRC_TRANS_COMPLETE |  \
                              (uint32_t)MCAN_INTR_SRC_TRANS_CANCEL_FINISH |  \
                              (uint32_t)MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG |  \
                              (uint32_t)MCAN_INTR_SRC_PROTOCOL_ERR_ARB |  \
                              (uint32_t)MCAN_INTR_SRC_PROTOCOL_ERR_DATA |  \
                              (uint32_t)MCAN_INTR_SRC_BUS_OFF_STATUS)

/*! \brief completion type for Tx in dma mode - Intermediate completion */
#define CANFD_DMA_TX_COMPLETION_INTERMEDIATE        (1U)
/*! \brief completion type for Tx in dma mode - Final completion */
#define CANFD_DMA_TX_COMPLETION_FINAL               (2U)
/*! \brief completion type for Rx in dma mode - Intermediate completion */
#define CANFD_DMA_RX_COMPLETION_INTERMEDIATE        (1U)
/*! \brief completion type for Rx in dma mode - Final completion */
#define CANFD_DMA_RX_COMPLETION_FINAL               (2U)

/** @addtogroup CANFD_DRIVER_INTERNAL_DATA_STRUCTURE
 @{ */

/**
 *  \brief    This enumeration defines the MCAN FIFO/Buffer element Size
 */
typedef enum CANFDLLD_MCANElemSize_t
{
    /*! 8 byte data field */
    CANFDLLD_MCANElemSize_8BYTES = 0U,

    /*! 12 byte data field */
    CANFDLLD_MCANElemSize_12BYTES = 1U,

    /*! 16 byte data field */
    CANFDLLD_MCANElemSize_16BYTES = 2U,

    /*! 20 byte data field */
    CANFDLLD_MCANElemSize_20BYTES = 3U,

    /*! 24 byte data field */
    CANFDLLD_MCANElemSize_24BYTES = 4U,

    /*! 32 byte data field */
    CANFDLLD_MCANElemSize_32BYTES = 5U,

    /*! 48 byte data field */
    CANFDLLD_MCANElemSize_48BYTES = 6U,

    /*! 64 byte data field */
    CANFDLLD_MCANElemSize_64BYTES = 7U
}CANFDLLD_MCANElemSize;

/*!
 *  \brief    This enumeration defines the values used to represent the CANFD driver state
 */
typedef enum CANFDLLD_DriverState_t
{
    /*! CANFD controller not initialized */
    CANFDLLD_DriverState_UNINIT,

    /*! CANFD controller started */
    CANFDLLD_DriverState_STARTED,

    /*! CANFD controller stopped */
    CANFDLLD_DriverState_STOPPED,

    /*! CANFD controller in sleep mode */
    CANFDLLD_DriverState_SLEEP
}CANFDLLD_DriverState;

/** @}*/

/** @addtogroup CANFD_DRIVER_EXTERNAL_DATA_STRUCTURE
 @{ */

/*!
 *  \brief    Enumerates the values used to represent the MCAN mode of operation
 */
typedef enum CANFDLLD_MCANOperationMode_t
{
    /*! MCAN normal mode */
    CANFDLLD_MCANOperationMode_NORMAL = 0U,

    /*! MCAN SW initialization mode */
    CANFDLLD_MCANOperationMode_SW_INIT = 1U
}CANFDLLD_MCANOperationMode;

/*!
 *  \brief    This enumeration defines the values used to set the direction of message object
 */
typedef enum CANFDLLD_Direction_t
{
    /*! Message object used to receive data */
    CANFDLLD_Direction_RX,

    /*! Message object used to transmit data */
    CANFDLLD_Direction_TX
} CANFDLLD_Direction;

/*!
 *  \brief    This enumeration defines the values used to represent the CAN Identifier Type
 */
typedef enum CANFDLLD_MCANXidType_t
{
    /*! 11bit MCAN Identifier */
    CANFDLLD_MCANXidType_11_BIT,

    /*! 29bit MCAN Identifier */
    CANFDLLD_MCANXidType_29_BIT
} CANFDLLD_MCANXidType;

/*!
 *  \brief    This enumeration defines the CAN frame type
 */
typedef enum CANFDLLD_MCANFrameType_t
{
    /*! Classic Frame */
    CANFDLLD_MCANFrameType_CLASSIC,

    /*! FD Frame */
    CANFDLLD_MCANFrameType_FD
} CANFDLLD_MCANFrameType;

/**
 *  \brief    This enumeration defines the MCAN timeout counter configuration
 */
typedef enum CANFDLLD_MCANTimeOutSelect_t
{
    /*! Continuous operation Mode */
    CANFDLLD_MCANTimeOutSelect_CONT = 0U,

    /*! Timeout controlled by Tx Event FIFO */
    CANFDLLD_MCANTimeOutSelect_TX_EVENT_FIFO = 1U,

    /*! Timeout controlled by Rx FIFO 0 */
    CANFDLLD_MCANTimeOutSelect_RX_FIFO0 = 2U,

    /*! Timeout controlled by Rx FIFO 1 */
    CANFDLLD_MCANTimeOutSelect_RX_FIFO1 = 3U
}CANFDLLD_MCANTimeOutSelect;

/**
 *  \brief    This enumeration defines the MCAN ECC Error Types
 */
typedef enum CANFDLLD_MCANECCErrType_t
{
    /*! ECC Single Error Correction */
    CANFDLLD_MCANECCErrType_SEC = 0U,

    /*! ECC Single Error Detection */
    CANFDLLD_MCANECCErrType_DED = 1U
}CANFDLLD_MCANECCErrType;

/**
 *  \brief    This enumeration defines the MCAN Loopback mode
 */
typedef enum CANFDLLD_MCANLoopBackMode_t
{
    /*! This mode can be used for hot self-test and this mode will not affect bus state. */
    CANFDLLD_MCANLoopBackMode_INTERNAL = 0U,

    /*! In this mode, MCAN the MCAN treats its own transmitted messages as received messages
     *  and stores them (if they pass acceptance filtering) into an Rx Buffer or an Rx FIFO.
     *  This mode will affect bus state.
     */
    CANFDLLD_MCANLoopBackMode_EXTERNAL = 1U
}CANFDLLD_MCANLoopBackMode;

/**
 *  \brief    This enumeration defines the MCAN's communication state
 */
typedef enum CANFDLLD_MCANCommState_t
{
    /*! MCAN is synchronizing on CANFD communication */
    CANFDLLD_MCANCommState_SYNCHRONIZING = 0U,

    /*! MCAN is neither receiver nor transmitter */
    CANFDLLD_MCANCommState_IDLE = 1U,

    /*! MCAN is operating as receiver */
    CANFDLLD_MCANCommState_RECEIVER = 2U,

    /*! MCAN is operating as transmitter */
    CANFDLLD_MCANCommState_TRANSMITTER = 3U
}CANFDLLD_MCANCommState;

/**
 *  \brief    This enumeration defines the  MCAN's Error Code
 */
typedef enum CANFDLLD_MCANErrCode_t
{
    /*! No error occurred since LEC has been reset by
     *  successful reception or transmission.
     */
    CANFDLLD_MCANErrCode_NO_ERROR = 0U,

    /*! More than 5 equal bits in a sequence have occurred in a part of
     *  a received message where this is not allowed.
     */
    CANFDLLD_MCANErrCode_STUFF_ERROR = 1U,

    /*! A fixed format part of a received frame has the wrong format. */
    CANFDLLD_MCANErrCode_FORM_ERROR = 2U,

    /*! The message transmitted by the MCAN was not acknowledged by another node. */
    CANFDLLD_MCANErrCode_ACK_ERROR = 3U,

    /*! During the transmission of a message (with the exception of
     *  the arbitration field), the device wanted to send a
     *  recessive level (bit of logical value 1),
     *  but the monitored bus value was dominant.
     */
    CANFDLLD_MCANErrCode_BIT1_ERROR = 4U,

    /*! During the transmission of a message (or acknowledge bit,
     *  or active error flag, or overload flag), the device wanted to send
     *  a dominant level (data or identifier bit logical value 0),
     *  but the monitored bus value was recessive. During Bus_Off recovery
     *  this status is set each time a sequence of 11 recessive bits has been
     *  monitored. This enables the CPU to monitor the proceeding of
     *  the Bus_Off recovery sequence (indicating the bus is not stuck at
     *  dominant or continuously disturbed).
     */
    CANFDLLD_MCANErrCode_BIT0_ERROR = 5U,

    /*! The CRC check sum of a received message was incorrect.
     *   The CRC of an incoming message does not match with the
     *   CRC calculated from the received data.
     */
    CANFDLLD_MCANErrCode_CRC_ERROR = 6U,

    /*! Any read access to the Protocol Status Register re-initializes the LEC to 7.
     * When the LEC shows the value 7, no CANFD bus event was detected since the last
     * CPU read access to the Protocol Status Register.
     */
    CANFDLLD_MCANErrCode_NO_CHANGE = 7U
}CANFDLLD_MCANErrCode;

/**
 *  \brief  This enumeration describes a list of all the reasons for which the driver will
 *          invoke application callback functions.
 */
typedef enum CANFDLLD_Reason_t
{
    /**
     * \brief  Data has been received and the application is required to read and process the data.
     */
    CANFDLLD_Reason_RX                         = 0x1,

    /**
     * \brief  Data has been succesfully transmitted.
     */
    CANFDLLD_Reason_TX_COMPLETION              = 0x2,

    /**
     * \brief  Data transmission is succesfully canceled.
     */
    CANFDLLD_Reason_TX_CANCELED                = 0x3,

    /**
     * \brief  Data has been succesfully transmitted.
     */
    CANFDLLD_Reason_ECC_ERROR                  = 0x4,

    /**
     * \brief  Bus Off condition detected.
     */
    CANFDLLD_Reason_BUSOFF                     = 0x5,

    /**
     * \brief  Protocol error in data phase detected.
     */
    CANFDLLD_Reason_PROTOCOL_ERR_DATA_PHASE    = 0x6,

    /**
     * \brief  Protocol error in arbitration phase detected.
     */
    CANFDLLD_Reason_PROTOCOL_ERR_ARB_PHASE     = 0x7
}CANFDLLD_Reason;

/*!
 *  \brief    This enumeration defines the values used to represent the GET/SET options
 *
 *  @sa
 *  CANFDLLD_OptionTLV
 */
typedef enum CANFDLLD_Option_t
{
    /*! Used to get the MCAN Tx and Rx error counters
     * @sa CANFD_lld_getOptions
     *
     * NOTE: The length in the TLV should be sizeof(CANFDLLD_MCANErrCntStatus) for this option.
     */
    CANFDLLD_Option_MCAN_ERROR_COUNTER,

    /*! Used to get the MCAN protocol status
     * @sa CANFD_lld_getOptions
     *
     * NOTE: The length in the TLV should be sizeof(CANFDLLD_MCANProtocolStatus) for this option.
     */
    CANFDLLD_Option_MCAN_PROTOCOL_STATUS,

    /*! Used to get the MCAN message object software maintained statistics
     * @sa CANFD_lld_getOptions
     *
     * NOTE: The length in the TLV should be sizeof(CANFDLLD_MCANMsgObjectStats) for this option.
     *      Application must fill in the message object handle for which the statistics is requested.
     *
     */
    CANFDLLD_Option_MCAN_MSG_OBJECT_STATS,

    /*! Used to put the MCAN module in init or operational state
     * @sa CANFD_lld_setOptions
     *
     * NOTE: The length in the TLV should be 1 byte for this option.
     * Valid values: Refer to (CANFDLLD_MCANOperationMode)
     */
    CANFDLLD_Option_MCAN_MODE,

    /*! Used to enable or disable internal/external loopback mode
     * @sa CANFD_lld_setOptions
     *
     * NOTE: The length in the TLV should be sizeof(CANFDLLD_MCANLoopbackCfgParams) for this option.
     *
     */
    CANFDLLD_Option_MCAN_LOOPBACK,

    /*! Used to request a local power down or wakeup from a local power down
     * @sa CANFD_lld_setOptions
     *
     * NOTE: The length in the TLV should be 1 byte for this option.
     * Valid values are
     *  1   -   MCAN Sleep
     *  0   -   MCAN Wakeup
     *
     */
    CANFDLLD_Option_MCAN_POWER_DOWN
} CANFDLLD_Option;

typedef void *CANFDLLD_DmaHandle;

typedef void *CANFDLLD_DmaChConfig;

/**
 * \brief  Data structure defines the MCAN Loopback parameters.
 */
typedef struct CANFDLLD_MCANLoopbackCfgParams_t
{
    /*! Enable or disable loopback mode
     * Valid values are
     *  0   -   Disable
     *  1   -   Enable
     */
    uint32_t                enable;

    /*! Loopback mode: Internal or External */
    CANFDLLD_MCANLoopBackMode  mode;
}CANFDLLD_MCANLoopbackCfgParams;

/**
 * \brief   Data structure defines the parameters for bit timing calculation.
 *          Bit timing related to data phase will be valid only in case where
 *          MCAN is put in CANFD mode and will be '0' otherwise.
 */
typedef struct CANFDLLD_MCANBitTimingParams_t
{
    /*! Nominal Baud Rate Pre-scaler */
    uint32_t        nomBrp;

    /*! NominalProp Segment value */
    uint32_t        nomPropSeg;

    /*! NominalPhase Segment1 value */
    uint32_t        nomPseg1;

    /*! NominalPhase Segment2 value */
    uint32_t        nomPseg2;

    /*! Nominal (Re)Synchronization Jump Width */
    uint32_t        nomSjw;

    /*! Nominal Baud Rate Pre-scaler */
    uint32_t        dataBrp;

    /*! NominalProp Segment value */
    uint32_t        dataPropSeg;

    /*! NominalPhase Segment1 value */
    uint32_t        dataPseg1;

    /*! NominalPhase Segment2 value */
    uint32_t        dataPseg2;

    /*! Nominal (Re)Synchronization Jump Width */
    uint32_t        dataSjw;
}CANFDLLD_MCANBitTimingParams;

/**
 * \brief  Data structure defines the MCAN Transmitter Delay Compensation parameters.
 */
typedef struct CANFDLLD_MCANTdcConfig_t
{
    /*! Transmitter Delay Compensation Filter Window Length
     *   Range: [0x0-0x7F]
     */
    uint32_t        tdcf;

    /*! Transmitter Delay Compensation Offset
     *   Range: [0x0-0x7F]
     */
    uint32_t        tdco;
}CANFDLLD_MCANTdcConfig;

/**
 * \brief   Data structure defines the MCAN Global Filter Configuration parameters.
 */
typedef struct CANFDLLD_MCANGlobalFiltConfig_t
{
    /*! Reject Remote Frames Extended
     *   0 = Filter remote frames with 29-bit extended IDs
     *   1 = Reject all remote frames with 29-bit extended IDs
     */
    uint32_t        rrfe;

    /*! Reject Remote Frames Standard
     *   0 = Filter remote frames with 11-bit standard IDs
     *   1 = Reject all remote frames with 11-bit standard IDs
     */
    uint32_t        rrfs;

    /*! Accept Non-matching Frames Extended
     *   0 = Accept in Rx FIFO 0
     *   1 = Accept in Rx FIFO 1
     *   others = Reject
     */
    uint32_t        anfe;

    /*! Accept Non-matching Frames Standard
     *   0 = Accept in Rx FIFO 0
     *   1 = Accept in Rx FIFO 1
     *   others = Reject
     */
    uint32_t        anfs;
}CANFDLLD_MCANGlobalFiltConfig;

/**
 * \brief   Data structure defines the MCAN Message RAM Configuration Parameters.
 *          Message RAM can contain following sections:
 *          Standard ID filters, Extended ID filters, TX FIFO(or TX Q),
 *          TX Buffers, TX EventFIFO, RX FIFO0, RX FIFO1, RX Buffer.
 *          Note: If particular section in the RAM is not used then it's size
 *          should be initialized to '0'
 *          (Number of buffers in case of Tx/Rx buffer).
 */
typedef struct CANFDLLD_MCANMsgRAMCfgParams_t
{
    /*! List Size: Standard ID
     *   0 = No standard Message ID filter
     *   1-127 = Number of standard Message ID filter elements
     *   others = Values greater than 128 are interpreted as 128
     */
    uint32_t            lss;

    /*! List Size: Extended ID
     *   0 = No standard Message ID filter
     *   1-64 = Number of standard Message ID filter elements
     *   others = Values greater than 64 are interpreted as 64
     */
    uint32_t            lse;

    /*! Number of Dedicated Transmit Buffers
     *   0 = No Dedicated Tx Buffers
     *   1-32 = Number of Dedicated Tx Buffers
     *   others = Values greater than 32 are interpreted as 32
     */
    uint32_t            txBufNum;

    /*! Transmit FIFO/Queue Size
     *   0 = No Tx FIFO/Queue
     *   1-32 = Number of Tx Buffers used for Tx FIFO/Queue
     *   others = Values greater than 32 are interpreted as 32
     */
    uint32_t            txFIFOSize;

    /*! Tx FIFO/Queue Mode
     *   0 = Tx FIFO operation
     *   1 = Tx Queue operation
     */
    uint32_t            txBufMode;

    /*! Event FIFO Size
     *   0 = Tx Event FIFO disabled
     *   1-32 = Number of Tx Event FIFO elements
     *   others = Values greater than 32 are interpreted as 32
     */
    uint32_t            txEventFIFOSize;

    /*! Tx Event FIFO Watermark
     *   0 = Watermark interrupt disabled
     *   1-32 = Level for Tx Event FIFO watermark interrupt
     *   others = Watermark interrupt disabled
     */
    uint32_t            txEventFIFOWaterMark;

    /*! Rx FIFO0 Size
     *   0 = No Rx FIFO
     *   1-64 = Number of Rx FIFO elements
     *   others = Values greater than 64 are interpreted as 64
     */
    uint32_t            rxFIFO0size;

    /*! Rx FIFO0 Watermark
     *   0 = Watermark interrupt disabled
     *   1-63 = Level for Rx FIFO 0 watermark interrupt
     *   others = Watermark interrupt disabled
     */
    uint32_t            rxFIFO0waterMark;

    /*! Rx FIFO0 Operation Mode
     *   0 = FIFO blocking mode
     *   1 = FIFO overwrite mode
     */
    uint32_t            rxFIFO0OpMode;

    /*! Rx FIFO1 Size
     *   0 = No Rx FIFO
     *   1-64 = Number of Rx FIFO elements
     *   others = Values greater than 64 are interpreted as 64
     */
    uint32_t            rxFIFO1size;

    /*! Rx FIFO1 Watermark
     *   0 = Watermark interrupt disabled
     *   1-63 = Level for Rx FIFO 1 watermark interrupt
     *   others = Watermark interrupt disabled
     */
    uint32_t            rxFIFO1waterMark;

    /*! Rx FIFO1 Operation Mode
     *   0 = FIFO blocking mode
     *   1 = FIFO overwrite mode
     */
    uint32_t            rxFIFO1OpMode;
}CANFDLLD_MCANMsgRAMCfgParams;

/**
 * \brief   Data structure defines the MCAN ECC configuration parameters.
 */
typedef struct CANFDLLD_MCANECCConfigParams_t
{
    /*! Enable/disable ECC
     *   0 = Disable ECC
     *   1 = Enable ECC
     */
    uint32_t            enable;

    /*! Enable/disable ECC Check
     *   0 = Disable ECC Check
     *   1 = Enable ECC Check
     */
    uint32_t            enableChk;

    /*! Enable/disable Read Modify Write operation
     *   0 = Disable Read Modify Write operation
     *   1 = Enable Read Modify Write operation
     */
    uint32_t            enableRdModWr;
}CANFDLLD_MCANECCConfigParams;

/**
 * \brief   Data structure defines the MCAN error logging counters status.
 */
typedef struct CANFDLLD_MCANErrCntStatus_t
{
    /*! Transmit Error Counter */
    uint32_t            transErrLogCnt;

    /*! Receive Error Counter */
    uint32_t            recErrCnt;

    /*! Receive Error Passive
     *   0 = The Receive Error Counter is below the error passive level(128)
     *   1 = The Receive Error Counter has reached the error passive level(128)
     */
    uint32_t            rpStatus;

    /*! CAN Error Logging */
    uint32_t            canErrLogCnt;
}CANFDLLD_MCANErrCntStatus;

/**
 * \brief   Data structure defines the MCAN protocol status.
 */
typedef struct CANFDLLD_MCANProtocolStatus_t
{
    /*! Last Error Code
     *   Refer enum #CANFDLLD_MCANErrCode
     */
    uint32_t                lastErrCode;

    /*! Activity - Monitors the module's CAN communication state.
     *   refer enum #CANFDLLD_MCANCommState
     */
    uint32_t                act;

    /*! Error Passive
     *   0 = The M_CAN is in the Error_Active state
     *   1 = The M_CAN is in the Error_Passive state
     */
    uint32_t                errPassive;

    /*! Warning Status
     *   0 = Both error counters are below the Error_Warning limit of 96
     *   1 = At least one of error counter has reached the Error_Warning
     *       limit of 96
     */
    uint32_t                warningStatus;

    /*! Bus_Off Status
     *   0 = The M_CAN is not Bus_Off
     *   1 = The M_CAN is in Bus_Off state
     */
    uint32_t                busOffStatus;

    /*! Data Phase Last Error Code
     *   Refer enum #CANFDLLD_MCANErrCode
     */
    uint32_t                dlec;

    /*! ESI flag of last received CAN FD Message
     *   0 = Last received CAN FD message did not have its ESI flag set
     *   1 = Last received CAN FD message had its ESI flag set
     */
    uint32_t                resi;

    /*! BRS flag of last received CAN FD Message
     *   0 = Last received CAN FD message did not have its BRS flag set
     *   1 = TLast received CAN FD message had its BRS flag set
     */
    uint32_t                rbrs;

    /*! Received a CAN FD Message
     *   0 = Since this bit was reset by the CPU, no CAN FD message has been
     *       received
     *   1 = Message in CAN FD format with FDF flag set has been received
     */
    uint32_t                rfdf;

    /*! Protocol Exception Event
     *   0 = No protocol exception event occurred since last read access
     *   1 = Protocol exception event occurred
     */
    uint32_t                pxe;

    /*! Transmitter Delay Compensation Value */
    uint32_t                tdcv;
}CANFDLLD_MCANProtocolStatus;

/**
 * \brief   Data structure defines the ECC Error forcing.
 */
typedef struct CANFDLLD_MCANECCErrForceParams_t
{
    /*! Error type to be forced
     *   Refer enum  #CANFDLLD_MCANECCErrType.
     */
    uint32_t                errType;

    /*! Row address where error needs to be applied. */
    uint32_t                rowNum;

    /*! Column/Data bit that needs to be flipped when
     *   force_sec or force_ded is set
     */
    uint32_t                bit1;

    /*! Data bit that needs to be flipped when force_ded is set */
    uint32_t                bit2;

    /*! Force Error once
     *   1: The error will inject an error to the specified row only once
     */
    uint32_t                errOnce;

    /*! Force error on the next RAM read */
    uint32_t                errForce;
}CANFDLLD_MCANECCErrForceParams;

/**
 * \brief   Data structure defines the ECC Error Status.
 */
typedef struct CANFDLLD_MCANECCErrStatus_t
{
    /*! Single Bit Error Status
     *   0 = No Single Bit Error pending
     *   1 = Single Bit Error pending
     */
    uint32_t                secErr;

    /*! Double Bit Error Status
     *   0 = No Double Bit Error pending
     *   1 = Double Bit Error pending
     */
    uint32_t                dedErr;

    /*! Indicates the row/address where the single or double bit
     *   error occurred.
     */
    uint32_t                row;

    /*! Indicates the bit position in the ram data that is in error
     */
    uint32_t                bit1;

    /*! Indicates the bit position in the ram data that is in error
     *   Valid only in case of DED.
     */
    uint32_t                bit2;
}CANFDLLD_MCANECCErrStatus;

/*!
 * \brief
 *  Response structure definition for Error and status information.
 */
typedef struct CANFDLLD_ErrStatusResp_t
{
    union
    {
        /*! ECC Error Status. */
        CANFDLLD_MCANECCErrStatus      eccErrStatus;

        /*! Protocol Status. */
        CANFDLLD_MCANProtocolStatus    protocolStatus;
    }u;
} CANFDLLD_ErrStatusResp;

/**
 * \brief   Data structure defines the MCAN initialization parameters.
 */
typedef struct CANFDLLD_MCANInitParams_t
{
    /*! FD Operation Enable
     *   0 = FD operation disabled
     *   1 = FD operation enabled
     */
    uint32_t                    fdMode;

    /*! Bit Rate Switch Enable
     *   This is valid only when opMode = 1.
     *   0 = Bit rate switching for transmissions disabled
     *   1 = Bit rate switching for transmissions enabled
     */
    uint32_t                    brsEnable;

    /*! Transmit Pause
     *   0 = Transmit pause disabled
     *   1 = Transmit pause enabled
     */
    uint32_t                    txpEnable;

    /*! FEdge Filtering during Bus Integration
     *   0 = Edge filtering disabled
     *   1 = Two consecutive dominant tq required to detect an edge for
     *       hard synchronization
     */
    uint32_t                    efbi;

    /*! Protocol Exception Handling Disable
     *   0 = Protocol exception handling enabled
     *   1 = Protocol exception handling disabled
     */
    uint32_t                    pxhddisable;

    /*! Disable Automatic Retransmission
     *   0 = Automatic retransmission of messages not transmitted successfully
     *       enabled
     *   1 = Automatic retransmission disabled
     */
    uint32_t                    darEnable;

    /*! Wakeup Request Enable
     *   0 = Wakeup request is disabled
     *   1 = Wakeup request is enabled
     */
    uint32_t                    wkupReqEnable;

    /*! Auto-Wakeup Enable
     *   0 = Auto-Wakeup is disabled
     *   1 = Auto-Wakeup is enabled
     */
    uint32_t                    autoWkupEnable;

    /*! Emulation/Debug Suspend Enable
     *   0 = Emulation/Debug Suspend is disabled
     *   1 = Emulation/Debug Suspend is enabled
     */
    uint32_t                    emulationEnable;

    /*! Emulation/Debug Suspend Fast Ack Enable
     *   0 = Emulation/Debug Suspend does not wait for idle/immediate effect
     *   1 = Emulation/Debug Suspend waits for idle/graceful stop
     */
    uint32_t                    emulationFAck;

    /*! Clock Stop Fast Ack Enable
     *   0 = Clock Stop does not wait for idle/immediate effect
     *   1 = Clock Stop waits for idle/graceful stop
     */
    uint32_t                    clkStopFAck;

    /*! Start value of the Message RAM Watchdog Counter
     *   Range:[0x0-0xFF]
     */
    uint32_t                    wdcPreload;

    /*! Transmitter Delay Compensation Enable
     *   0 = Transmitter Delay Compensation is disabled
     *   1 = Transmitter Delay Compensation is enabled
     */
    uint32_t                    tdcEnable;

    /*! Transmitter Delay Compensation parameters.
     *   Refer struct #CANFDLLD_MCANTdcConfig.
     */
    CANFDLLD_MCANTdcConfig         tdcConfig;

    /*! Bus Monitoring Mode
     *   0 = Bus Monitoring Mode is disabled
     *   1 = Bus Monitoring Mode is enabled
     */
    uint32_t                    monEnable;

    /*! Restricted Operation Mode
     *   0 = Normal CAN operation
     *   1 = Restricted Operation Mode active
     *   This mode should not be combined with test modes.
     */
    uint32_t                    asmEnable;

    /*! Timestamp Counter Prescaler.
     *   Range:[0x0-0xF]
     */
    uint32_t                    tsPrescalar;

    /*! Timeout source selection.
     *   00b: Timestamp counter value always 0x0000
     *   01b: Timestamp counter value incremented according to tsPrescalar
     *   10b: External timestamp counter value used
     *   11b: Same as 00b
     */
    uint32_t                    tsSelect;

    /*! Time-out counter source select.
     *   Refer enum #CANFDLLD_MCANTimeOutSelect.
     */
    CANFDLLD_MCANTimeOutSelect     timeoutSelect;

    /*! Start value of the Timeout Counter (down-counter).
     *   The Timeout Counter is decremented in multiples of CAN bit times [1-16]
     *   depending on the configuration of the tsPrescalar.
     *   Range: [0x0-0xFFFF]
     */
    uint32_t                    timeoutPreload;

    /*! Timeout Counter Enable
     *   0 - Timeout Counter is disabled
     *   1 - Timeout Counter is enabled
     */
    uint32_t                    timeoutCntEnable;

    /*! Global Filter Configuration parameters.
     *    Refer struct #CANFDLLD_MCANGlobalFiltConfig.
     */
    CANFDLLD_MCANGlobalFiltConfig  filterConfig;

    /*! Message RAM Configuration parameters.
     *    Refer struct #CANFDLLD_MCANMsgRAMCfgParams.
     */
    CANFDLLD_MCANMsgRAMCfgParams   msgRAMConfig;

    /*! ECC Configuration parameters.
     *    Refer struct #CANFDLLD_MCANECCConfigParams.
     */
    CANFDLLD_MCANECCConfigParams   eccConfig;

    /*! Enable/Disable error/status interrupts
     * Note: Must be enabled to receive error and status interrupts. */
    uint32_t                    errInterruptEnable;

    /*! Enable/Disable data interrupts.
     * Note: Must be enabled to receive transmit complete and data receive interrupts. */
    uint32_t                    dataInterruptEnable;
}CANFDLLD_MCANInitParams;

/**
 * \brief
 *  Options TLV data structure
 *
 * @details
 *  Specifies the option type, length, value.
 */
typedef struct CANFDLLD_OptionTLV_t
{
    /**
     * \brief   Option Name
     */
    CANFDLLD_Option            type;

    /**
     * \brief   Option Length
     */
    int32_t                 length;

    /**
     * \brief   Option Value
     */
    void*                   value;
}CANFDLLD_OptionTLV;

/**
 * \brief
 *  CANFD Master Control Block
 *
 * @details
 *  The structure describes the CANFD Driver and is used to hold the relevant
 *  information with respect to the CANFD module.
 */
typedef struct CANFDLLD_Object_t
{
    /*!
     * \brief   Base address of the register address space to be used.
     */
    uint32_t                regBaseAddress;

    /**
     * \brief   CANFD driver internal state
     */
    CANFDLLD_DriverState               state;

    /**
     * \brief   CANFD driver init parameters
     */
    CANFDLLD_MCANInitParams            initParams;

    /**
     * \brief   Data Length to DLC mapping
     */
    uint8_t                         mcanDataSize[16];

    /**
     * \brief   Message object handle book keeping
     */
    struct CANFDLLD_MessageObject_t*  msgObjectHandle[MCAN_MAX_MSG_OBJECTS];

    /**
     * \brief   Tx Mapping to message handles for transmit post processing
     */
    struct CANFDLLD_MessageObject_t*   txMapping[MCAN_MAX_TX_MSG_OBJECTS];

    /**
     * \brief   Rx Mapping to message handles for Rx processing
     */
    struct CANFDLLD_MessageObject_t*   rxMapping[MCAN_MAX_RX_MSG_OBJECTS];

    /**
     * \brief   Number of error and status interrupts received
     */
    uint32_t                        errStatusInterrupts;

    /**
     * \brief   Number of interrupts received for message Tx or Rx
     */
    uint32_t                        interrupts;

    /**
     * \brief   Number of ECC interrupts received
     */
    uint32_t                        eccInterrupts;

    /**
     * \brief   Number of Bus-Off interrupts received
     */
    uint32_t                        busOffInterrupts;

    /**
     * \brief   Number of Protocol error in data phase interrupts received
     */
    uint32_t                        protoDataErrInterrupts;

    /**
     * \brief   Number of Protocol error in arbitration phase interrupts received
     */
    uint32_t                        protoArbErrInterrupts;

    /**
     * \brief   Tx Status of the message object
     */
    uint8_t                         txStatus[MCAN_MAX_TX_MSG_OBJECTS];

    /**
     * \brief   Flag to toggle the usage of FIFO 0 and FIFO 1. Valid values are 0 and 1.
     */
    uint32_t                        useFifoNum;

    /**
     * \brief   Buffer used to read message RAM.
     */
    MCAN_RxBufElement               rxBuffElem;

    /**
     * \brief   Dma driver handle.
     */
    CANFDLLD_DmaHandle             canfdDmaHandle;

    /**
     * \brief   Pointer to Dma channel configuration.
     */
    CANFDLLD_DmaChConfig           canfdDmaChCfg;

    /**
     * \brief   Pointer to be used by application to store miscellaneous data.
     */
    void*                          args;
}CANFDLLD_Object;

/**
 * \brief
 *  CANFD DMA message configuration used for Tx
 */
typedef struct CANFDLLD_DmaMsgConfig_s
{
    /**
     * \brief   data length in Bytes for every message.
     */
    uint32_t        dataLengthPerMsg;
    /**
     * \brief   Number of messages.
     */
    uint32_t        numMsgs;
    /**
     * \brief   pointer to the data buffer. This should a 2d array of uint8[numMsgs][dataLengthPerMsg]
     */
    const void      *data;
    /**
     * \brief   Used by driver to store current message count out of the numMsgs already processed.
     */
    uint32_t        currentMsgNum;
}CANFDLLD_DmaMsgConfig;

/**
 * \brief
 *  CANFD Rx Buffer used in DMA mode
 *
 * @details
 *  This structure defines the Rx buffer format used in DMA mode.
 *  This matches with the Rx Buffer format in message ram. The Dma will be
 *  configured to read the full Rx buffer with header from message ram.
 */
typedef struct CANFDLLD_DmaRxBuf_s
{
    /* Header word 1 in Rx buffer in message ram */
    uint32_t        header1;
    /* Header word 2 in Rx buffer in message ram */
    uint32_t        header2;
    /* Data Bytes Rx buffer in message ram */
    uint8_t         data[64];
}CANFDLLD_DmaRxBuf;

/**
 * \brief
 *  CAN message object block
 *
 * @details
 *  The structure defines the message object
 */
typedef struct CANFDLLD_MessageObject_t
{
    /**
     * \brief   Starting range of the Message Id to which the configuration belongs.
     * For Tx and single Message Id objects the startMsgId = endMsgId.
     */
    uint32_t                startMsgId;

    /**
     * \brief   Ending range of the Message Id to which the configuration belongs
     * For Tx and single Message Id objects the startMsgId = endMsgId.
     */
    uint32_t                endMsgId;

    /**
     * \brief   Pointer to the CANFD driver object
     */
    CANFDLLD_Object*           ptrCanFdObj;

    /**
     * \brief   Message object direction.
     */
    CANFDLLD_Direction         direction;

    /**
     * \brief   Message object type.
     */
    CANFDLLD_MCANXidType       msgIdType;

    /**
     * \brief   Allocated message object number
     */
    uint32_t                messageObjNum;

    /**
     * \brief   Tx buffer number used to send data
     */
    uint32_t                txElement;

    /**
     * \brief   Rx buffer number used to receive data
     */
    uint32_t                rxElement;

    /**
     * \brief   Part of message ram to accessed by this message object. Refer enum #MCAN_MemType.
     */
    MCAN_MemType            memType;

    /**
     * \brief   Number of interrupts received
     */
    uint32_t                interruptsRxed;

    /**
     * \brief   Number of messages processed
     */
    uint32_t                messageProcessed;

    /**
     * \brief   Enable dma mode for this message object
     */
    uint32_t                enableDmaMode;

    /**
     * \brief   Dma Event number allocated for this message object
     */
    uint32_t                dmaEventNo;

    /**
     * \brief   Dma message configuration
     */
    CANFDLLD_DmaMsgConfig   dmaMsgConfig;

}CANFDLLD_MessageObject;

/*!
 *  \brief      CANFD module handle returned by the CANFD_lld_init() API call.
 */
typedef CANFDLLD_Object* CANFDLLD_Handle;

/*!
 *  \brief      CANFD message object handle returned by the CANFD_lld_createMsgObject() API call.
 */
typedef CANFDLLD_MessageObject* CANFDLLD_MsgObjHandle;

/*!
 *  \brief    Data structure defines the software maintained message object statistics.
 */
typedef struct CANFDLLD_MCANMsgObjectStats_t
{
    /*! Message Object Handle for which the statistics is requested */
    CANFDLLD_MsgObjHandle      handle;

    /*! Message Object direction */
    uint32_t                direction;

    /*! Starting range of the Message Id to which the configuration belongs.
     * For Tx and single Message Id objects the startMsgIdentifier = endMsgIdentifier */
    uint32_t                startMsgIdentifier;

    /*! Ending range of the Message Id to which the configuration belongs
     * For Tx and single Message Id objects the startMsgIdentifier = endMsgIdentifier. */
    uint32_t                endMsgIdentifier;

    /*! Number of interrupts received */
    uint32_t                interruptsRxed;

    /*! Number of messages processed */
    uint32_t                messageProcessed;
} CANFDLLD_MCANMsgObjectStats;

/** @}*/

/** @addtogroup CANFD_DRIVER_EXTERNAL_FUNCTION
 @{ */

/**
 *  @b Description
 *  @n
 *      Function initializes the CANFD driver with the specified hardware attributes.
 *      It resets and configures the MCAN module, sets up the Message RAM and ECC Aggregator.
 *      It configures the CANFD driver with the control parameters.
 *
 *  \param  hCanfd
 *      CANFD handle of type CANFDLLD_Handle 
 *
 *  \return
 *      Success -   Handle to the CANFD Driver
 */

int32_t CANFD_lld_init(CANFDLLD_Handle hCanfd);

/**
 *  @b Description
 *  @n
 *      Function closes the CANFD driver instance and cleanups all the memory allocated by the CANFD driver.
 *
 *  \param  handle
 *      Handle to the CANFD Driver
 *
 *  \return
 *      Success     - 0
 *  \return
 *      Error       - <0
 */

int32_t CANFD_lld_deInit(CANFDLLD_Handle handle);

/**
 *  @b Description
 *  @n
 *      Function configures the bit time parameters for the CANFD module.
 *
 *  \param  handle
 *      Handle to the CANFD Driver
 *  \param  bitTimeParams
 *      Bit time configuration parameters
 *
 *  \return
 *      Success     - 0
 *  \return
 *      Error       - <0
 */
int32_t CANFD_lld_configBitTime(CANFDLLD_Handle handle, const CANFDLLD_MCANBitTimingParams* bitTimeParams);

/**
 *  @b Description
 *  @n
 *      Function configures the receive or transmit message object.
 *      It also enables Tx completion and Tx cancelation interrupts .
 *      The callback function will be invoked on data transmit complete for transmit message objects
 *      OR
 *      upon receiving data for receive message objects. The application MUST then call CANFD_lld_read() API to process the received data.
 *
 *  \param  handle
 *      Handle to the CANFD Driver
 *  \param  ptrCanMsgObj
 *      Message Object configuration parameters
 *
 *  \return
 *      Success -   Handle to the message object.
 *  \return
 *      Error   -   NULL
 */
int32_t CANFD_lld_createMsgObject(CANFDLLD_Handle handle, CANFDLLD_MessageObject* ptrCanMsgObj);

/**
 *  @b Description
 *  @n
 *      Function configures a receive message objects for a range of message identifiers.
 *      It also enables Rx interrupts.
 *      The callback function will be invoked upon receiving data for receive message objects.
 *      The application MUST then call CANFD_lld_read() API to process the received data.
 *
 *  \param  handle
 *      Handle to the CANFD Driver
 *  \param  ptrCanMsgObj
 *      Message Object configuration parameters
 *
 *  \return
 *      Success -   Handle to the message object.
 *  \return
 *      Error   -   NULL
 */
int32_t CANFD_lld_createRxRangeMsgObject(CANFDLLD_Handle handle, CANFDLLD_MessageObject* ptrCanMsgObj);

/**
 *  @b Description
 *  @n
 *      Function deletes a message object.
 *
 *  \param  handle
 *      Handle to the message object
 *
 *  \return
 *      Success     - 0
 *  \return
 *      Error       - <0
 */
int32_t CANFD_lld_deleteMsgObject(CANFDLLD_MsgObjHandle handle);

/**
 *  @b Description
 *  @n
 *      Function used by the application to transmit data.
 *
 *  \param  handle
 *      Handle to the message object
 *  \param  id
 *      Message Identifier
 *  \param  frameType
 *      Frame type - Classic or FD
 *  \param  dataLength
 *      Data Length to be transmitted.
 *      Valid values: 1 to 64 bytes.
 *  \param  data
 *      Data to be transmitted
 *
 *  \return
 *      Success     - 0
 *  \return
 *      Error       - <0
 */
int32_t CANFD_lld_write(CANFDLLD_MsgObjHandle handle, uint32_t id, CANFDLLD_MCANFrameType frameType, uint32_t dataLength, const uint8_t* data);

/**
 *  @b Description
 *  @n
 *      Function used by the application to cancel a pending data transmit.
 *
 *  \param  handle
 *      Handle to the message object
 *
 *  \return
 *      Success     - 0
 *  \return
 *      Error       - <0
 */
int32_t CANFD_lld_writeCancel(CANFDLLD_MsgObjHandle handle);

/**
 *  @b Description
 *  @n
 *      Function used by the application to initiate transmit data in dma mode.
 *      DMA mode is recommended to be used when multiple msgs needs to be transmitted.
 *      This will transmit first msg and configure the dma to copy subsequent msgs in message ram.
 *      Once first msg transfer is completed the CANFD_dmaTxCompletionCallback function is called.
 *      Application needs to call the API CANFD_lld_writeDmaTriggerNext to trigger transmission
 *      of subsequent msgs.
 *
 *  \param  handle
 *      Handle to the message object
 *  \param  id
 *      Message Identifier
 *  \param  frameType
 *      Frame type - Classic or FD
 *  \param  dataLengthPerMsg
 *      Data Length to be transmitted per msg.
 *      Valid values: 1 to 64 bytes.
 *  \param  numMsgs
 *      Number of msgs to be transmitted
 *  \param  data
 *      Data to be transmitted. This should be a 2d array of uint8[numMsgs][dataLengthPerMsg]
 *
 *  \return
 *      Success     - 0
 *  \return
 *      Error       - <0
 */
int32_t CANFD_lld_writeDma(CANFDLLD_MsgObjHandle handle, uint32_t id, CANFDLLD_MCANFrameType frameType, uint32_t dataLengthPerMsg, uint32_t numMsgs, void* data);

/**
 *  @b Description
 *  @n
 *      Function used by the application to strat transmission of next msg in dma mode.
 *      Transfer should be initiated using the API CANFD_lld_writeDma before calling this function.
 *      This should be called after the previous transfer is completed and CANFD_dmaTxCompletionCallback
 *      is called.
 *
 *  \param  handle
 *      Handle to the message object
 *
 *  \return
 *      Success     - 0
 *  \return
 *      Error       - <0
 */
int32_t CANFD_lld_writeDmaTriggerNext(CANFDLLD_MsgObjHandle handle);

/**
 *  @b Description
 *  @n
 *      Function used by the application to get the filter element number.
 *
 *  \param  eventNum
 *      DMA event number
 *
 *  \return
 *      value to be programmed in CAN msgID filter with DAM event number
 */
uint32_t CANFD_lld_getFilterEventConfig(uint32_t eventNum);

/**
 *  @b Description
 *  @n
 *      Function is used by the application to get the CAN message from message RAM using a receive message object.
 *      NOTE: This API must ONLY be called from the callback context.
 *
 *  \param  handle
 *      Handle to the message object
 *  \param  id
 *      Message Identifier
 *  \param  ptrFrameType
 *      Frame type - Classic or FD
 *  \param  idType
 *      Meassage Id type - 11 bit standard or 29 bit extended
 *  \param  ptrDataLength
 *      Data Length of the received frame.
 *      Valid values: 1 to 64 bytes.
 *  \param  data
 *      Received data.
 *
 *  \return
 *      Success     - 0
 *  \return
 *      Error       - <0
 */
int32_t CANFD_lld_read(CANFDLLD_MsgObjHandle handle, uint32_t* id, CANFDLLD_MCANFrameType* ptrFrameType, CANFDLLD_MCANXidType* idType, uint32_t* ptrDataLength, uint8_t* data);

/**
 *  @b Description
 *  @n
 *      Function is used by the application to configure reading the received msgs from message ram.
 *      This API will configure the DMA to copy msg from MCAN message ram to application buffer.
 *      For every message received the CANFD_dmaRxCompletionCallback will be called by driver by pointing
 *      to the latest msg in the data buffer.
 *
 *  \param  handle
 *      Handle to the message object
 *  \param  data
 *      Array of CANFDLLD_DmaRxBuf to receive the data from message ram. This should be an array
 *      of type CANFDLLD_DmaRxBuf and length numDmaRxBuf
 *  \param  numDmaRxBuf
 *      Number of dma buffers to read.
 *
 *  \return
 *      Success     - 0
 *  \return
 *      Error       - <0
 */
int32_t CANFD_lld_readDmaConfig(CANFDLLD_MsgObjHandle handle, void* data, uint32_t numDmaRxBuf);

/**
 *  @b Description
 *  @n
 *      Function is used by the application to get the error and status information from the driver.
 *
 *  \param  handle
 *      Handle to the CANFD Driver
 *  \param ptrOptInfo
 *      Option info in TLV format which is populated with the requested information
 *
 *  \return
 *      Success     - 0
 *  \return
 *      Error       - <0
 */
int32_t CANFD_lld_getOptions(CANFDLLD_Handle handle, CANFDLLD_OptionTLV* ptrOptInfo);

/**
 *  @b Description
 *  @n
 *      Function is used by the application to configure the driver options.
 *
 *  \param  handle
 *      Handle to the CANFD Driver
 *  \param ptrOptInfo
 *      Option info in TLV format which is used to configure the driver
 *
 *  \return
 *      Success     - 0
 *  \return
 *      Error       - <0
 */
int32_t CANFD_lld_setOptions(CANFDLLD_Handle handle, const CANFDLLD_OptionTLV* ptrOptInfo);

/**
 *  @b Description
 *  @n
 *      The function is the registered interrupt 0 ISR for the CANFD Driver.
 *
 *  \param  ptrCanFdObj
 *      Handle to the CANFD Driver
 *
 *  \return
 *      Not applicable
 */
void CANFD_lld_int0Isr (CANFDLLD_Object* ptrCanFdObj);

/**
 *  @b Description
 *  @n
 *      The function is the registered interrupt 1 ISR for the CANFD Driver.
 *
 *  \param  ptrCanFdObj
 *      Handle to the CANFD Driver
 *
 *  \return
 *      Not applicable
 */
void CANFD_lld_int1Isr (CANFDLLD_Object* ptrCanFdObj);

/**
 * \brief
 * Application specified callback function which is invoked
 * by the CANFD driver once transmit is complete or data has been received for the
 * specified message object.
 *
 *  \param  handle
 *      Message object handle for which the callback function is invoked.
 *  \param  reason
 *      Cause of the interrupt which prompted the callback.
 *
 */
ATTRIBUTE_WEAK void CANFD_lld_dataAppCallBack(CANFDLLD_MsgObjHandle handle, CANFDLLD_Reason reason);

/**
 * \brief
 * Application specified callback function which is invoked
 * by the CANFD driver on error or status change.
 *
 *  \param  handle Handle to the CANFD Driver
 *  \param  reason Cause of the interrupt which prompted the callback.
 *  \param  errStatusResp
 *      Response structure populated with the value of the fields that caused the error or status interrupt.
 *      Processing of this structure is dependent on the callback reason.
 *
 */
ATTRIBUTE_WEAK void CANFD_lld_errStatusAppCallBack(CANFDLLD_Handle handle, CANFDLLD_Reason reason, CANFDLLD_ErrStatusResp* errStatusResp);

/**
 * \brief API to open an CANFD DMA channel
 *
 * This API will open a DMA Channel using the appropriate DMA driver callbacks and the registered via Sysconfig
 *
 * \param canfdHandle CANFD Handle
 * \param dmaChCfg CANFD Channel config
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_lld_dmaOpen(CANFDLLD_Handle canfdHandle, CANFDLLD_DmaChConfig dmaChCfg);

/**
 * \brief API to close an CANFD DMA channel
 *
 * \param canfdHandle CANFD handle returned from \ref CANFD_lld_init
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_lld_dmaClose(CANFDLLD_Handle canfdHandle);

/**
 * \brief API to configure dma for the Tx message object. Called from the API CANFD_lld_createMsgObject.
 *
 * \param ptrCanFdObj pointer to the CANFD driver object
 * \param ptrCanMsgObj pointer to the CANFD message object
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_lld_createDmaTxMsgObject(CANFDLLD_Object *ptrCanFdObj, CANFDLLD_MessageObject* ptrCanMsgObj);

/**
 * \brief API to delete dma configuration for the Tx message object. Called from the API CANFD_lld_deleteMsgObject.
 *
 * \param ptrCanFdObj pointer to the CANFD driver object
 * \param ptrCanMsgObj pointer to the CANFD message object
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_lld_deleteDmaTxMsgObject(CANFDLLD_Object *ptrCanFdObj, CANFDLLD_MessageObject* ptrCanMsgObj);

/**
 * \brief API to enable dma event transfer for the Tx. Called from the API CANFD_lld_writeDma.
 *
 * \param ptrCanFdObj       pointer to the CANFD driver object
 * \param ptrCanMsgObj      pointer to the CANFD message object
 * \param dataLengthPerMsg  data length in bytes for each message
 * \param numMsgs           Number f messages to transmit
 * \param data              pointer to the data buffer to transmit
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_lld_configureDmaTx(CANFDLLD_Object *ptrCanFdObj, CANFDLLD_MessageObject* ptrCanMsgObj, uint32_t dataLengthPerMsg, uint32_t numMsgs, const void* data);

/**
 * \brief API to disbale dma event transfer for the Tx to cancel the transfer.
 *
 * \param ptrCanFdObj       pointer to the CANFD driver object
 * \param ptrCanMsgObj      pointer to the CANFD message object
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_lld_cancelDmaTx(CANFDLLD_Object *ptrCanFdObj, CANFDLLD_MessageObject* ptrCanMsgObj);

/**
 * \brief Callback function for the Tx completion.
 *        This is called for each message in the array of msgs provided in CANFD_lld_configureDmaTx.
 *        data will point to the current transmitted message.
 *        For intermediate message transfer completion the completionType will be set to CANFD_DMA_TX_COMPLETION_INTERMEDIATE
 *        for last message transfer completion the completionType will be set to CANFD_DMA_TX_COMPLETION_FINAL
 *
 * \param  ptrCanMsgObj   pointer to the CANFD message object
 * \param  data           pointer to current completed message
 * \param  completionType specifie completion type for the callback
 *
 */
ATTRIBUTE_WEAK void CANFD_dmaTxCompletionCallback(CANFDLLD_MessageObject* ptrCanMsgObj, void *data, uint32_t completionType);

/**
 * \brief API to configure dma for the Rx message object. Called from the CANFD_lld_createMsgObject.
 *
 * \param ptrCanFdObj   pointer to the CANFD driver object
 * \param ptrCanMsgObj  pointer to the CANFD message object
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_lld_createDmaRxMsgObject(CANFDLLD_Object *ptrCanFdObj, CANFDLLD_MessageObject* ptrCanMsgObj);

/**
 * \brief API to delete dma configuration for the Rx message object. Called from the CANFD_lld_deleteMsgObject.
 *
 * \param ptrCanFdObj   pointer to the CANFD driver object
 * \param ptrCanMsgObj  pointer to the CANFD message object
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_lld_deleteDmaRxMsgObject(CANFDLLD_Object *ptrCanFdObj, CANFDLLD_MessageObject* ptrCanMsgObj);

/**
 * \brief API to enable dma event transfer for the Rx. Called from the API CANFD_lld_readDma.
 *
 * \param ptrCanFdObj        pointer to the CANFD driver object
 * \param ptrCanMsgObj       pointer to the CANFD message object
 * \param dataLengthPerMsg   data length in bytes for each message
 * \param numMsgs            Number of messages to transmit
 * \param data               pointer to the data buffer to transmit
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_lld_configureDmaRx(CANFDLLD_Object *ptrCanFdObj, CANFDLLD_MessageObject* ptrCanMsgObj, uint32_t dataLengthPerMsg, uint32_t numMsgs, const void* data);

/**
 * \brief Callback function for the Rx completion.
 *        This is called for each message in the array of msgs provided in CANFD_lld_configureDmaRx.
 *        data will point to the current received message.
 *        For intermediate message transfer completion the completionType will be set to CANFD_DMA_RX_COMPLETION_INTERMEDIATE
 *        for last message transfer completion the completionType will be set to CANFD_DMA_RX_COMPLETION_FINAL
 *
 * \param  ptrCanMsgObj    pointer to the CANFD message object
 * \param  data            pointer to current recieved message
 * \param  completionType  specifie completion type for the callback
 *
 */
ATTRIBUTE_WEAK void CANFD_dmaRxCompletionCallback(CANFDLLD_MessageObject* ptrCanMsgObj, void *data, uint32_t completionType);

/** @}*/

#ifdef __cplusplus
}
#endif

#endif /* #ifndef CANFD_LLD_H_ */


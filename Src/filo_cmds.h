/*
**             Copyright 2017 by Kvaser AB, Molndal, Sweden
**                         http://www.kvaser.com
**
** This software is dual licensed under the following two licenses:
** BSD-new and GPLv2. You may use either one. See the included
** COPYING file for details.
**
** License: BSD-new
** ==============================================================================
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the <organization> nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
** ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
** LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
** CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
** SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
** BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
** IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
** ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
** POSSIBILITY OF SUCH DAMAGE.
**
**
** License: GPLv2
** ==============================================================================
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
**
**
** IMPORTANT NOTICE:
** ==============================================================================
** This source code is made available for free, as an open license, by Kvaser AB,
** for use with its applications. Kvaser AB does not accept any liability
** whatsoever for any third party patent or other immaterial property rights
** violations that may result from any usage of this source code, regardless of
** the combination of source code and various applications that it can be used
** in, or with.
**
** -----------------------------------------------------------------------------
*/

/*
** Description:
**   Definitions for Filo and  Memorator II
** -----------------------------------------------------------------------------
*/

#ifndef FILO_CMDS_H_
#define FILO_CMDS_H_

#pragma pack(push, 1)
#include "compilerassert.h"

#include <stdint.h>

#ifdef FILO_PRIVATE
#   include "filo_private.h"
#endif

#define CMD_RX_STD_MESSAGE                    12
#define CMD_TX_STD_MESSAGE                    13
#define CMD_RX_EXT_MESSAGE                    14
#define CMD_TX_EXT_MESSAGE                    15
#define CMD_SET_BUSPARAMS_REQ                 16
#define CMD_GET_BUSPARAMS_REQ                 17
#define CMD_GET_BUSPARAMS_RESP                18
#define CMD_GET_CHIP_STATE_REQ                19
#define CMD_CHIP_STATE_EVENT                  20
#define CMD_SET_DRIVERMODE_REQ                21
#define CMD_GET_DRIVERMODE_REQ                22
#define CMD_GET_DRIVERMODE_RESP               23
#define CMD_RESET_CHIP_REQ                    24
#define CMD_RESET_CARD_REQ                    25
#define CMD_START_CHIP_REQ                    26
#define CMD_START_CHIP_RESP                   27
#define CMD_STOP_CHIP_REQ                     28
#define CMD_STOP_CHIP_RESP                    29
#define CMD_READ_CLOCK_REQ                    30
#define CMD_READ_CLOCK_RESP                   31
#define CMD_GET_CARD_INFO_2                   32
// 33 may be used
#define CMD_GET_CARD_INFO_REQ                 34
#define CMD_GET_CARD_INFO_RESP                35
#define CMD_GET_INTERFACE_INFO_REQ            36
#define CMD_GET_INTERFACE_INFO_RESP           37
#define CMD_GET_SOFTWARE_INFO_REQ             38
#define CMD_GET_SOFTWARE_INFO_RESP            39
#define CMD_GET_BUSLOAD_REQ                   40
#define CMD_GET_BUSLOAD_RESP                  41
#define CMD_RESET_STATISTICS                  42
#define CMD_CHECK_LICENSE_REQ                 43
#define CMD_CHECK_LICENSE_RESP                44
#define CMD_ERROR_EVENT                       45
// 46, 47 reserved
#define CMD_FLUSH_QUEUE                       48
#define CMD_RESET_ERROR_COUNTER               49
#define CMD_TX_ACKNOWLEDGE                    50
#define CMD_CAN_ERROR_EVENT                   51

#define CMD_MEMO_GET_DATA                     52
#define CMD_MEMO_PUT_DATA                     53
#define CMD_MEMO_PUT_DATA_START               54
#define CMD_MEMO_ASYNCOP_START                55
#define CMD_MEMO_ASYNCOP_GET_DATA             56
#define CMD_MEMO_ASYNCOP_CANCEL               57
#define CMD_MEMO_ASYNCOP_FINISHED             58
#define CMD_DISK_FULL_INFO                    59
#define CMD_TX_REQUEST                        60
#define CMD_SET_HEARTBEAT_RATE_REQ            61
#define CMD_HEARTBEAT_RESP                    62
#define CMD_SET_AUTO_TX_BUFFER                63
#define CMD_GET_EXTENDED_INFO                 64
#define CMD_TCP_KEEPALIVE                     65
#define CMD_TX_INTERVAL_REQ                   66
#define CMD_TX_INTERVAL_RESP                  67
#define CMD_FILO_FLUSH_QUEUE_RESP             68
// 69-72 can be reused
#define CMD_AUTO_TX_BUFFER_REQ                72
#define CMD_AUTO_TX_BUFFER_RESP               73
#define CMD_SET_TRANSCEIVER_MODE_REQ          74
#define CMD_TREF_SOFNR                        75
#define CMD_SOFTSYNC_ONOFF                    76
#define CMD_USB_THROTTLE                      77
#define CMD_SOUND                             78
#define CMD_LOG_TRIG_STARTUP                  79
#define CMD_SELF_TEST_REQ                     80
#define CMD_SELF_TEST_RESP                    81
#define CMD_USB_THROTTLE_SCALED               82

// 83-85 can be reused

#define CMD_SET_IO_PORTS_REQ                  86
#define CMD_GET_IO_PORTS_REQ                  87
#define CMD_GET_IO_PORTS_RESP                 88
// 89-94 can be used
#define CMD_GET_CAPABILITIES_REQ              95
#define CMD_GET_CAPABILITIES_RESP             96
#define CMD_GET_TRANSCEIVER_INFO_REQ          97
#define CMD_GET_TRANSCEIVER_INFO_RESP         98
#define CMD_MEMO_CONFIG_MODE                  99
// 100 can be used
#define CMD_LED_ACTION_REQ                   101
#define CMD_LED_ACTION_RESP                  102
#define CMD_INTERNAL_DUMMY                   103
#define CMD_READ_USER_PARAMETER              104
#define CMD_MEMO_CPLD_PRG                    105
#define CMD_LOG_MESSAGE                      106
#define CMD_LOG_TRIG                         107
#define CMD_LOG_RTC_TIME                     108

#define CMD_SCRIPT_ENVVAR_CTRL_REQ           109
#define CMD_SCRIPT_ENVVAR_CTRL_RESP          110

#define CMD_SCRIPT_ENVVAR_TRANSFER_CTRL_REQ  111 // PC wants to set value in VM
#define CMD_SCRIPT_ENVVAR_TRANSFER_CTRL_RESP 112 // PC wants to set value in VM
#define CMD_SCRIPT_ENVVAR_TRANSFER_BULK      113 // PC wants to set value in VM

#define CMD_SCRIPT_CTRL_REQ                  116
#define CMD_SCRIPT_CTRL_RESP                 117

#define CMD_SCRIPT_ENVVAR_NOTIFY_EVENT       118
#define CMD_GET_DEVICE_NAME_REQ              119
#define CMD_GET_DEVICE_NAME_RESP             120

#define CMD_PING_REQ                         121
#define CMD_PING_RESP                        122

#define CMD_SET_UNLOCK_CODE                  123
#define CMD_WRITE_USER_PARAMETER             124
#define CMD_CONFUSED_RESP                    125

#define MEMO_STATUS_SUCCESS                    0
#define MEMO_STATUS_MORE_DATA                  1
#define MEMO_STATUS_UNKNOWN_COMMAND            2
#define MEMO_STATUS_FAILED                     3 // .data[] contains more info
#define MEMO_STATUS_EOF                        4

#define WRITE_USER_PARAM_TYPE_LICENSE_DATA     0
#define WRITE_USER_PARAM_TYPE_LICENSE_CHECK    1
#define WRITE_USER_PARAM_TYPE_LICENSE_COMMIT   2

#if defined(FEATURE_LWIP)
# if FEATURE_LWIP
#  define LWIP_FIXLEN(x) (sizeof(filoResponse))
# else
#  define LWIP_FIXLEN(x) (x)
# endif
#else
# define LWIP_FIXLEN(x) (x)
#endif

typedef struct {
  uint8_t d[24];
} data24_t;

// These are the diagnostics data sent via CMD_TCP_KEEPALIVE.
typedef struct {
  uint8_t version;
  union {
    struct {
      uint8_t  medium;
      uint8_t  tx_queue_max;
      int8_t   rssi_min;
      uint16_t rx_wnd_min;
      uint16_t tx_wnd_min;
      uint16_t rtt_max;
    } v1;
    struct {
      uint8_t  d[15];
    } diagRawData16_t;
  };
} Diagnostics;

#define DIAGNOSTICS_VERSION                    1

#define SCRIPT_CTRL_ERR_SUCCESS                0
#define SCRIPT_CTRL_ERR_NO_MORE_PROCESSES      1
#define SCRIPT_CTRL_ERR_FILE_NOT_FOUND         2
#define SCRIPT_CTRL_ERR_OPEN_FILE_ERR          3
#define SCRIPT_CTRL_ERR_OPEN_FILE_NO_MEM       4
#define SCRIPT_CTRL_ERR_FILE_READ_ERR          5
#define SCRIPT_CTRL_ERR_LOAD_FILE_ERR          6
#define SCRIPT_CTRL_ERR_OUT_OF_CODE_MEM        7
#define SCRIPT_CTRL_ERR_FILE_REWIND_FAIL       8
#define SCRIPT_CTRL_ERR_LOAD_FAIL              9
#define SCRIPT_CTRL_ERR_SETUP_FAIL            10
#define SCRIPT_CTRL_ERR_SETUP_FUN_TABLE_FAIL  11
#define SCRIPT_CTRL_ERR_SETUP_PARAMS_FAIL     12
#define SCRIPT_CTRL_ERR_PROCESSES_NOT_FOUND   13
#define SCRIPT_CTRL_ERR_START_FAILED          14
#define SCRIPT_CTRL_ERR_STOP_FAILED           15
#define SCRIPT_CTRL_ERR_SPI_BUSY              16
#define SCRIPT_CTRL_ERR_PROCESS_NOT_STOPPED   17
#define SCRIPT_CTRL_ERR_PROCESS_NOT_RUNNING   18
#define SCRIPT_CTRL_ERR_ENVVAR_NOT_FOUND      19

#define SCRIPT_CTRL_ERR_UNKNOWN_COMMAND       20
#define SCRIPT_CTRL_ERR_PROCESS_NOT_LOADED    21
#define SCRIPT_CTRL_ERR_COMPILER_VERSION      22
#define SCRIPT_CTRL_ERR_INVALID_PARAMETER     23

#define SCRIPT_CTRL_ERR_NOT_IMPLEMENTED       43


// cmdScriptCtrlReXX payload union
typedef union
{
  struct {                      // SCRIPT_CMD_SCRIPT_EVENT
    uint32_t eventNo;
    uint32_t eventType;
    uint32_t eventData;
  } cmdScriptEvent;

  struct {
    uint8_t  d[20];
  } cmdRawData20_t;

  struct {                      // SCRIPT_CMD_SCRIPT_QUERY_STATUS
    uint32_t scriptStatus;
  } cmdScriptInfo;

} scriptCtrlPayload;


typedef struct {
  uint8_t           cmdLen;
  uint8_t           cmdNo;
  uint8_t           scriptNo;   // Which script to deliver command to
  uint8_t           channel;
  uint32_t          subCmd;     // Command specifys what the data contains
  scriptCtrlPayload payload;    // Data associated with event
} cmdScriptCtrlReq;

typedef struct {
  uint8_t           cmdLen;
  uint8_t           cmdNo;
  uint8_t           scriptNo;
  uint8_t           reserved;
  uint32_t          subCmd;
  uint32_t          status;
  scriptCtrlPayload payload;
} cmdScriptCtrlResp;


#define M32_ENVVAR_GET_INFO                    1
#define M32_ENVVAR_GET_MAX_SIZE                2

#define M32_ENVVAR_TYPE_UNKNOWN               -1
#define M32_ENVVAR_TYPE_INT                    1
#define M32_ENVVAR_TYPE_FLOAT                  2
#define M32_ENVVAR_TYPE_STRING                 3

// Union to transfer data to/from device in cmdEnvvarCtrlReXXX
typedef union {
  struct {
    uint32_t hash;
    int32_t  type;
    int32_t  length;
  } cmdGetInfo;                 // M32_ENVVAR_GET_INFO
  struct {
    int32_t  maxLength;
  } cmdMaxLength;               // M32_ENVVAR_GET_MAX_SIZE
} envvarData;


typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  scriptNo;            // Which script to deliver command to
  uint8_t  channel;
  uint32_t subCmd;              // Command
  uint8_t  data[24];
} cmdEnvvarCtrlReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  scriptNo;            // Which script to deliver command to
  uint8_t  channel;
  uint32_t subCmd;              // Command
  uint32_t status;
  uint8_t  data[20];
} cmdEnvvarCtrlResp;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  subCmd;
  uint8_t  reserved1;
  uint32_t data1;               // Sector number for disk reads
  uint16_t data2;               // Sector count for disk reads
  uint8_t  data[22];
} cmdMemoGetDataReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  subCmd;
  uint8_t  dataLen;
  uint16_t offset;
  uint8_t  status;              // MEMO_STATUS_xxx
  uint8_t  reserved;
  uint8_t  data[24];            // Data, or status (dioStat in 0 and lioStat in 1)
} cmdMemoGetDataResp;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  subCmd;
  uint8_t  reserved1;
  uint32_t data1;               // Sector number for disk reads
  uint16_t data2;               // Sector count for disk reads
  uint8_t  reserved2[14];
} cmdMemoPutDataStartReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  subCmd;
  uint8_t  dataLen;
  uint16_t offset;
  uint16_t reserved;
  uint8_t  data[24];
} cmdMemoPutDataReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  subCmd;
  uint8_t  dataLen;
  uint8_t  status;
  uint8_t  padding2;
  uint16_t reserved;
  uint8_t  data[24];            // Data, or status (dioStat in 0 and lioStat in 1)
} cmdMemoPutDataResp;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  subCmd;
  uint8_t  reserved1;
  uint16_t reserved2;
} cmdMemoAsyncopStartReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  subCmd;
  uint8_t  status;
  uint16_t reserved[2];
} cmdMemoAsyncopStartResp;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  subCmd;
  uint8_t  status;
  uint16_t reserved[2];
} cmdMemoAsyncopFinishedResp;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  subCmd;
  uint8_t  reserved1;
  uint16_t reserved2;
} cmdMemoAsyncopGetDataReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  subCmd;
  uint8_t  status;
  uint8_t  reserved[4];
  uint8_t  data[24];
} cmdMemoAsyncopGetDataResp;

typedef struct {
  uint8_t   cmdLen;
  uint8_t   cmdNo;
  uint8_t   subCmd;
  uint8_t   reserved1;
  uint16_t  reserved2;
} cmdMemoAsyncopCancelReq;


// *All* subcommands to be in one number series
#define MEMO_SUBCMD_GET_FS_INFO                1 // Get DOS filesys info; for get_data
#define MEMO_SUBCMD_GET_DISK_INFO_A            2 // Get disk info; for get_data
#define MEMO_SUBCMD_GET_DISK_INFO_B            3 // Get logio info; for get_data

#define MEMO_SUBCMD_READ_PHYSICAL_SECTOR       4
#define MEMO_SUBCMD_WRITE_PHYSICAL_SECTOR      5
#define MEMO_SUBCMD_ERASE_PHYSICAL_SECTOR      6
#define MEMO_SUBCMD_READ_LOGICAL_SECTOR        7
#define MEMO_SUBCMD_WRITE_LOGICAL_SECTOR       8
#define MEMO_SUBCMD_ERASE_LOGICAL_SECTOR       9

#define MEMO_SUBCMD_FORMAT_DISK               10 // Format disk (FAT16 or -32) asyncop
#define MEMO_SUBCMD_INIT_DISK                 11 // Create logdata.kmf, asyncop
#define MEMO_SUBCMD_CLEAR_DATA                12 // Clear logdata.kmf, asyncop

#define MEMO_SUBCMD_GET_MISC_INFO             13 // for get_data
#define MEMO_SUBCMD_GET_RTC_INFO              14 // for get_data
#define MEMO_SUBCMD_PUT_RTC_INFO              15 // for put_data

#define MEMO_SUBCMD_GET_FS_INFO_B             16 // Get various filesystem info

#define MEMO_SUBCMD_FASTREAD_PHYSICAL_SECTOR  17
#define MEMO_SUBCMD_FASTREAD_LOGICAL_SECTOR   18

#define MEMO_SUBCMD_OPEN_FILE                 19
#define MEMO_SUBCMD_READ_FILE                 20
#define MEMO_SUBCMD_CLOSE_FILE                21
#define MEMO_SUBCMD_WRITE_FILE                22
#define MEMO_SUBCMD_DELETE_FILE               23


// Structs used to transfer data to and from the firmware
// Use CMD_MEMO_GET_DATA and ..PUT_DATA to send and receive these.
// They are identified by MEMO_SUBCMD_... constants
// Don't forget to put a CompilerAssert at the end of this file for
// each new struct you define.
//

// Max length 24 bytes
typedef struct {
  uint16_t fat_size;            // Size of the FAT, in sectors
  uint16_t fat_type;            // 12 or 16 depending on FAT type.
  uint16_t dir_entries;         // Number of directory entries in the root dir
  uint16_t cluster_size;        // Two-logarithm of the cluster size in sectors
  uint32_t fat1_start;          // First FAT starts in this sector
  uint32_t first_data_sector;   // First sector available for data
  uint32_t last_data_sector;    // Last sector available for data
  uint32_t logfile_sectors;     // Number of sectors in log file
} MemoDataFsInfo;

// Max length 24 bytes
typedef struct {
  uint32_t first_param_sector;  // First sector for param.lif
  uint32_t last_param_sector;   // Last sector for param.lif
  uint32_t first_dbase_sector;  // First sector for databases (if any)
  uint32_t last_dbase_sector;   // Last sector for databases (if any)
  uint32_t reserved0;
  uint32_t reserved1;
} MemoDataFsInfoB;

// Max length 24 bytes
typedef struct {
  uint8_t  productRev;
  uint8_t  oemId[2];
  uint8_t  reserved1;
  char     productName[10];
  uint16_t dateCode;
  uint32_t mfgId;
  uint32_t serialNumber;
} MemoDataDiskInfoA;

// Max length 24 bytes
typedef struct {
  uint8_t  disk_type;
  uint8_t  version;
  uint8_t  read_time;
  uint8_t  wr_factor;
  uint8_t  file_format;
  uint8_t  erase_value;
  uint16_t read_blk_size;
  uint16_t wr_blk_size;
  uint16_t trans_speed;
  uint32_t data_size;
  uint8_t  reserved[8];
} MemoDataDiskInfoB;


// Power flags, suitably abstracted - do not necessarily map directly
// to the hardware signals in Memo2, but should be usable in the future
// even with new, as yet unknown hardware.
#define MEMO_POWER_BAT_FAULT                0x01 // Battery fault of some kind
#define MEMO_POWER_BAT_CHARGING             0x02 // Battery is charging
#define MEMO_POWER_BAT_POWER_OK             0x04 // Battery power OK
#define MEMO_POWER_EXTPOWER_OK              0x08 // External power OK
#define MEMO_POWER_USBPOWER_OK              0x10 // USB power OK

#define MEMO_TEMPERATURE_FAILURE               0 // Failed measuring temperature
#define MEMO_TEMPERATURE_MEMO2                 1 // Temperature is encoded as in Memo2 (NTC 10K connected to 3.3V)

// Max length 24 bytes
typedef struct {
  uint8_t  diskPresent;
  uint8_t  configMode;
  uint8_t  diskWriteProtected;
  uint8_t  temperatureEncoding; // Encoding of 'temperature' field
  uint16_t powerStatus;         // MEMO_POWER_xxx flags
  uint16_t temperature;         // Temperature, raw value
  uint8_t  cpldVersion;         // CPLD version number
  uint8_t  reserved[15];
} MemoDataMiscInfo;


// Max length 24 bytes
typedef struct {
  uint8_t  second;
  uint8_t  minute;
  uint8_t  hour;
  uint8_t  day;
  uint8_t  month;
  uint8_t  year;
  uint16_t padding2;
} MemoDataRtcInfo;

// Max length 24 bytes
typedef struct {
  uint32_t maxDataSize;
  uint32_t dbaseSpace;
  uint32_t reserveSpace;
  uint8_t  fileSystem;
  uint8_t  reserved[11];
} MemoInitDiskReq;

// Max length 24 bytes
typedef struct {
  uint8_t dioStatus;
  uint8_t lioStatus;
  uint8_t reserved[10];
} MemoInitDiskResp;

// The data field in a cmdLogMessage with MSGFLAG_ERROR_FRAME flag set
// carries this data inside it.
typedef struct canErrorFrameData_s {
  uint8_t busStatus;
  uint8_t errorFactor;
  uint8_t txErrorCounter;
  uint8_t rxErrorCounter;
} canErrorFrameData_t;





#define SOUND_SUBCOMMAND_INIT                  0
#define SOUND_SUBCOMMAND_BEEP                  1
#define SOUND_SUBCOMMAND_NOTE                  2
#define SOUND_SUBCOMMAND_DISABLE               3

#define LED_SUBCOMMAND_ALL_LEDS_ON             0
#define LED_SUBCOMMAND_ALL_LEDS_OFF            1
#define LED_SUBCOMMAND_LED_0_ON                2
#define LED_SUBCOMMAND_LED_0_OFF               3
#define LED_SUBCOMMAND_LED_1_ON                4
#define LED_SUBCOMMAND_LED_1_OFF               5
#define LED_SUBCOMMAND_LED_2_ON                6
#define LED_SUBCOMMAND_LED_2_OFF               7
#define LED_SUBCOMMAND_LED_3_ON                8
#define LED_SUBCOMMAND_LED_3_OFF               9
#define LED_SUBCOMMAND_LED_4_ON               10
#define LED_SUBCOMMAND_LED_4_OFF              11
#define LED_SUBCOMMAND_LED_5_ON               12
#define LED_SUBCOMMAND_LED_5_OFF              13
#define LED_SUBCOMMAND_LED_6_ON               14
#define LED_SUBCOMMAND_LED_6_OFF              15


#define CONFIG_DATA_CHUNK                     24


//===========================================================================
// Flags
//===========================================================================

//////////////////////////
// CAN message flags
#define MSGFLAG_ERROR_FRAME                 0x01 // Msg is a bus error
#define MSGFLAG_OVERRUN                     0x02 // Msgs following this has been lost
#define MSGFLAG_NERR                        0x04 // NERR active during this msg
#define MSGFLAG_WAKEUP                      0x08 // Msg rcv'd in wakeup mode
#define MSGFLAG_REMOTE_FRAME                0x10 // Msg is a remote frame
#define MSGFLAG_RESERVED_1                  0x20 // Reserved for future usage
#define MSGFLAG_TX                          0x40 // TX acknowledge
#define MSGFLAG_TXRQ                        0x80 // TX request


// This one is added to the identifier to mean an extended CAN identifier
// #define DRIVER_EXT_FLAG             0x80000000


/////////////////////////
// Chip status flags
#define BUSSTAT_BUSOFF                      0x01
#define BUSSTAT_ERROR_PASSIVE               0x02
#define BUSSTAT_ERROR_WARNING               0x04
#define BUSSTAT_ERROR_ACTIVE                0x08
#define BUSSTAT_BUSOFF_RECOVERY             0x10
#define BUSSTAT_IGNORING_ERRORS             0x20


////////////////////////
// Driver modes
#define DRIVERMODE_NORMAL                   0x01
#define DRIVERMODE_SILENT                   0x02
#define DRIVERMODE_SELFRECEPTION            0x03
#define DRIVERMODE_OFF                      0x04

////////////////////////
// Transceiver (logical) types. Must be the same as TRANSCEIVER_xxx in e.g. canlib.h.
#define FILO_TRANSCEIVER_TYPE_UNKNOWN          0
#define FILO_TRANSCEIVER_TYPE_251              1
#define FILO_TRANSCEIVER_TYPE_252              2 // 252/1053/1054 w/o opto
#define FILO_TRANSCEIVER_TYPE_SWC              6 // J2411 type
#define FILO_TRANSCEIVER_TYPE_1054_OPTO       11 // 1054 with optical isolation
#define FILO_TRANSCEIVER_TYPE_SWC_OPTO        12 // J2411 type with optical isolation
#define FILO_TRANSCEIVER_TYPE_1050            14 // TJA1050
#define FILO_TRANSCEIVER_TYPE_1050_OPTO       15 // TJA1050 with optical isolation
#define FILO_TRANSCEIVER_TYPE_LIN             19 // LIN
#define FILO_TRANSCEIVER_TYPE_KLINE           10 // K-Line

// Transceiver line modes. Must be the same as in canlib.h
#define FILO_TRANSCEIVER_LINEMODE_NA           0 // Not Affected/Not available.
#define FILO_TRANSCEIVER_LINEMODE_SWC_SLEEP    4 // SWC Sleep Mode.
#define FILO_TRANSCEIVER_LINEMODE_SWC_NORMAL   5 // SWC Normal Mode.
#define FILO_TRANSCEIVER_LINEMODE_SWC_FAST     6 // SWC High-Speed Mode.
#define FILO_TRANSCEIVER_LINEMODE_SWC_WAKEUP   7 // SWC Wakeup Mode.
#define FILO_TRANSCEIVER_LINEMODE_SLEEP        8 // Sleep mode for those supporting it.
#define FILO_TRANSCEIVER_LINEMODE_NORMAL       9 // Normal mode (the inverse of sleep mode) for those supporting it.
#define FILO_TRANSCEIVER_LINEMODE_STDBY       10 // Standby for those who support it
// Transceiver resnet modes. Not supported.
#define FILO_TRANSCEIVER_RESNET_NA             0


////////////////////////////
// Error codes.
// Used in CMD_ERROR_EVENT.
#define FIRMWARE_ERR_OK                        0 // No error.
#define FIRMWARE_ERR_CAN                       1 // CAN error, addInfo1 contains error code.
#define FIRMWARE_ERR_NVRAM_ERROR               2 // Flash error
#define FIRMWARE_ERR_NOPRIV                    3 // No privilege for attempted operation
#define FIRMWARE_ERR_ILLEGAL_ADDRESS           4 // Illegal RAM/ROM address specified
#define FIRMWARE_ERR_UNKNOWN_CMD               5 // Unknown command or subcommand
#define FIRMWARE_ERR_FATAL                     6 // A severe error. addInfo1 contains error code.
#define FIRMWARE_ERR_CHECKSUM_ERROR            7 // Downloaded code checksum mismatch
#define FIRMWARE_ERR_QUEUE_LEVEL               8 // Tx queue levels (probably driver error)
#define FIRMWARE_ERR_PARAMETER                 9 // Parameter error, addInfo1 contains offending command

////////////////////////////
// Error codes for EXTINFO status
#define LEAF_EXTINFO_STAT_OK                   0
#define LEAF_EXTINFO_STAT_FAILURE              1
#define LEAF_EXTINFO_NOT_IMPLEMENTED          15 // intentionally same as CANIO_NOT_IMPLEMENTED

// Maximum length of a command. Do not change this.
#define MAX_CMD_LEN                           32


// For CMD_READ_CLOCK_REQ
#define READ_CLOCK_NOW                      0x01 // Provide a fast, unsynchronized response.

// For CMD_GET_SOFTWARE_OPTIONS (max 32 flags here)
#define SWOPTION_CONFIG_MODE               0x01L // Memorator in config mode.
#define SWOPTION_AUTO_TX_BUFFER            0x02L // Firmware has auto tx buffers
#define SWOPTION_BETA                      0x04L // Firmware is a beta release
#define SWOPTION_RC                        0x08L // Firmware is a release candidate
#define SWOPTION_BAD_MOOD                  0x10L // Firmware detected config error or the like
#define SWOPTION_CPU_FQ_MASK               0x60L
#define SWOPTION_16_MHZ_CLK                0x00L // hires timers run at 16 MHZ
#define SWOPTION_32_MHZ_CLK                0x20L // hires timers run at 32 MHZ
#define SWOPTION_24_MHZ_CLK                0x40L // hires timers run at 24 MHZ
#define SWOPTION_XX_MHZ_CLK                0x60L // hires timers run at xx MHZ
#define SWOPTION_TIMEOFFSET_VALID          0x80L // the timeOffset field in txAcks and logMessages is vaild
#define SWOPTION_CAP_REQ                 0x1000L // Firmware supports CMD_GET_CAPABILITIES_REQ


// CMD_SET_AUTO_TX_REQ and _RESP enum values
#define AUTOTXBUFFER_CMD_GET_INFO              1 // Get implementation information
#define AUTOTXBUFFER_CMD_CLEAR_ALL             2 // Clear all buffers on a channel
#define AUTOTXBUFFER_CMD_ACTIVATE              3 // Activate a specific buffer
#define AUTOTXBUFFER_CMD_DEACTIVATE            4 // Dectivate a specific buffer
#define AUTOTXBUFFER_CMD_SET_INTERVAL          5 // Set tx buffer transmission interval
#define AUTOTXBUFFER_CMD_GENERATE_BURST        6 // Generate a burst of messages
#define AUTOTXBUFFER_CMD_SET_MSG_COUNT         7 // Set tx buffer message count

// CMD_SET_AUTO_TX_RESP bit values for automatic tx buffer capabilities
#define AUTOTXBUFFER_CAP_TIMED_TX           0x01 // Periodic transmission
#define AUTOTXBUFFER_CAP_AUTO_RESP_DATA     0x02 // Auto response to data frames
#define AUTOTXBUFFER_CAP_AUTO_RESP_RTR      0x04 // Auto response to RTR

// Use these message flags with cmdSetAutoTxBuffer.flags
#define AUTOTXBUFFER_MSG_REMOTE_FRAME       0x10 // Msg is a remote frame
#define AUTOTXBUFFER_MSG_EXT                0x80 // Extended identifier

// For CMD_SOFTSYNC_ONOFF
#define SOFTSYNC_OFF                           0
#define SOFTSYNC_ON                            1
#define SOFTSYNC_NOT_STARTED                   2

// For canTimeStampRef in cmdGetCardInfoResp
#define CAN_TIME_STAMP_REF_ACK                 0
#define CAN_TIME_STAMP_REF_SOF                 1
#define CAN_TIME_STAMP_REF_INTERRUPT           2
#define CAN_TIME_STAMP_REF_CANTIMER            3


// for script control
#define SCRIPT_CMD_SCRIPT_START              0x1
#define SCRIPT_CMD_SCRIPT_STOP               0x2
#define SCRIPT_CMD_SCRIPT_KILL               0x3
#define SCRIPT_CMD_SCRIPT_EVENT              0x4
#define SCRIPT_CMD_SCRIPT_LOAD               0x5
#define SCRIPT_CMD_SCRIPT_QUERY_STATUS       0x6

// script status info
// SCRIPT_CMD_SCRIPT_QUERY_STATUS
#define SCRIPT_STATUS_LOADED                 0x1
#define SCRIPT_STATUS_RUNNING                0x2


/*
** Command Structs
*/

// Header for every command.
typedef struct {
  uint8_t cmdLen; // The length of the whole packet (i.e. including this byte)
  uint8_t cmdNo;
} cmdHead;


//#define TIMESTAMP_AS_LONG(x) (*(uint32_t*) x.time)

/*
** Keep the following structs 4 byte aligned (to allow for future
** DPRAM access over the PCI bus.)
*/

// NOTE: cmdLogMessage struct is hardcoded in rawData2MsgXXX
//       in can_handler_2ch.c

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  channel;
  uint8_t  flags;               // MSGFLAG_*
  uint16_t time[3];
  uint8_t  dlc;
  uint8_t  timeOffset;
  uint32_t id;                  // incl. CAN_IDENT_IS_EXTENDED
  uint8_t  data[8];
} cmdLogMessage;

typedef struct
{
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  type;
  uint8_t  padding;
  uint16_t trigNo;              // A bit mask showing which trigger was activated.
  uint16_t time[3];
  uint32_t preTrigger;
  uint32_t postTrigger;
} cmdLogTrig;

typedef struct
{
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  verMajor;            // File version number
  uint8_t  verMinor;            // File version number
  uint16_t date;                // Real time date (YMD)
  uint16_t time;                // Real time time (HMS)
  uint8_t  canDriverType0;      // Transceiver type for channel 0.
  uint8_t  canDriverType1;      // Transceiver type for channel 1.
  uint8_t  hiresTimerFqMHz;     // High-resolution timer frequency, in MHz
  uint8_t  hardwareType;        // Our HWTYPE_xxx.
  uint16_t bitrate0L;           // Bit rate channel 0, low word
  uint8_t  bitrate0H;           // Bit rate channel 0, high byte
  uint8_t  padding1;
  uint16_t bitrate1L;           // Bit rate channel 1, low word
  uint8_t  bitrate1H;           // Bit rate channel 1, high byte
  uint8_t  padding2;
} cmdLogRtcTime;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  channel;
  uint8_t  flags;
  uint16_t time[3];
  uint8_t  rawMessage[14];
} cmdRxCanMessage;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  channel;
  uint8_t  transId;
  uint8_t  rawMessage[14];
  uint8_t  _padding0;
  uint8_t  flags;
} cmdTxCanMessage;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  channel;
  uint8_t  transId;
  uint16_t time[3];
  uint8_t  flags;               // Flags detected during tx - currently only NERR
  uint8_t  timeOffset;
} cmdTxAck;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  channel;
  uint8_t  transId;
  uint16_t time[3];
  uint16_t padding;
} cmdTxRequest;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  channel;
  uint32_t bitRate;
  uint8_t  tseg1;
  uint8_t  tseg2;
  uint8_t  sjw;
  uint8_t  noSamp;
} cmdSetBusparamsReq;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t channel;
} cmdGetBusparamsReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  channel;
  uint32_t bitRate;
  uint8_t  tseg1;
  uint8_t  tseg2;
  uint8_t  sjw;
  uint8_t  noSamp;
} cmdGetBusparamsResp;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t channel;
} cmdGetChipStateReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  channel;
  uint16_t time[3];
  uint8_t  txErrorCounter;
  uint8_t  rxErrorCounter;
  uint8_t  busStatus;
  uint8_t  padding;
  uint16_t padding2;
} cmdChipStateEvent;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  channel;
  uint8_t  driverMode;
  uint8_t  padding;
  uint16_t padding2;
} cmdSetDrivermodeReq;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t channel;
} cmdGetDrivermodeReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  channel;
  uint8_t  driverMode;
  uint8_t  padding;
  uint16_t padding2;
} cmdGetDrivermodeResp;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t channel;
} cmdResetChipReq;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t padding;
} cmdResetCardReq;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t channel;
} cmdStartChipReq;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t channel;
} cmdStartChipResp;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t channel;
} cmdStopChipReq;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t channel;
} cmdStopChipResp;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t flags;
} cmdReadClockReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  padding;
  uint16_t time[3];
  uint16_t padding2;
} cmdReadClockResp;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t padding;
} cmdSelfTestReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  padding;
  uint32_t results;
} cmdSelfTestResp;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  int8_t dataLevel;
} cmdGetCardInfo2Req;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  reserved0;           // Unused right now
  uint8_t  pcb_id[24];
  uint32_t oem_unlock_code;
} cmdGetCardInfo2Resp;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  int8_t  dataLevel;
} cmdGetCardInfoReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  channelCount;
  uint32_t serialNumber;
  uint32_t padding1;            // Unused right now
  uint32_t clockResolution;
  uint32_t mfgDate;
  uint8_t  EAN[8];              // LSB..MSB, then the check digit.
  uint8_t  hwRevision;
  uint8_t  usbHsMode;
  uint8_t  hwType;              // HWTYPE_xxx (only f/w 1.2 and after)
  uint8_t  canTimeStampRef;
} cmdGetCardInfoResp;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t channel;
} cmdGetInterfaceInfoReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  channel;
  uint32_t channelCapabilities;
  uint8_t  canChipType;
  uint8_t  canChipSubType;
  uint16_t padding;
} cmdGetInterfaceInfoResp;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t padding;
} cmdGetSoftwareInfoReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  padding0;
  uint32_t swOptions;
  uint32_t firmwareVersion;
  uint16_t maxOutstandingTx;
  uint16_t padding1;            // Currently unused
  uint32_t padding[4];          // Currently unused
} cmdGetSoftwareInfoResp;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t channel;
} cmdGetBusLoadReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  channel;
  uint16_t time[3];             // "Absolute" timestamp
  uint16_t sample_interval;     // Sampling interval in microseconds
  uint16_t active_samples;      // Number of samples where tx or rx was active
  uint16_t delta_t;             // Milliseconds since last response
} cmdGetBusLoadResp;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t channel;
} cmdResetStatisticsReq;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t padding;
} cmdCheckLicenseReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  padding;
  uint32_t licenseMask;
  uint32_t kvaserLicenseMask;
} cmdCheckLicenseResp;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  errorCode;
  uint16_t time[3];
  uint16_t padding;
  uint16_t addInfo1;
  uint16_t addInfo2;
} cmdErrorEvent;


typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  channel;
  uint8_t  flags;
  uint8_t  padding;
  uint16_t padding2;
} cmdFlushQueue;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  channel;
  uint8_t  flags;
  uint8_t  padding;
  uint16_t padding2;
} cmdFlushQueueResp;


typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t padding;
} cmdNoCommand;


typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t channel;
} cmdResetErrorCounter;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  flags;               // Currently only NERR (.._ERROR_FRAME is implicit)
  uint16_t time[3];
  uint8_t  channel;
  uint8_t  padding;
  uint8_t  txErrorCounter;
  uint8_t  rxErrorCounter;
  uint8_t  busStatus;
  uint8_t  errorFactor;
} cmdCanErrorEvent;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  channel;
  uint8_t  lineMode;
  uint8_t  resistorNet;         // Not used in Filo (left-over from LAPcan)
  uint16_t padding;
} cmdSetTransceiverModeReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint16_t time;
} cmdInternalDummy;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t status;
} cmdMemoCpldPrgResp;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t subCmd;
  uint8_t data[CONFIG_DATA_CHUNK];
} cmdMemoCpldPrgReq;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t subCmd;
  int16_t timeout;
  int16_t padding;
} cmdLedActionReq;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t subCmd;
} cmdLedActionResp;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t info;
} cmdDiskFullInfo;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t userNo;
  uint8_t paramNo;
  uint8_t status;
  uint8_t padding[3];
  uint8_t data[8];
} cmdReadUserParameter;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t userNo;
  uint8_t paramNo;
  uint8_t status;
  uint8_t type;
  union {
    struct {
      uint8_t pad[2];
      uint8_t clearBits[8];
      uint8_t setBits[8];
    } typeLicense;
    struct {
      uint8_t pad[2];
      uint8_t hash[16];
    } typeCheck;
  };
} cmdWriteUserParameter;


typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t reserved;
  int32_t interval;             // Config mode timeout in ms
  uint8_t padding[4];
} cmdMemoConfigModeReq;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t padding;
  uint8_t diskStat;             // true if the disk is there.
  uint8_t configMode;           // true if we're in config mode.
  uint8_t reserved[10];
} cmdMemoConfigModeResp;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  subCmd;
  uint16_t freq;
  uint16_t duration;
  uint32_t padding[2];
}  cmdSound;


typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  portNo;              // Hardware-specific port #
  uint32_t portVal;             // Hardware-specific port value
} cmdSetIoPortsReq;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t transId;
  uint8_t portNo;               // Hardware-specific port #
} cmdGetIoPortsReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  portNo;              // Hardware-specific port #
  uint32_t portVal;             // Hardware-specific port value
  uint32_t padding;
  uint16_t status;
  uint16_t padding2;
} cmdGetIoPortsResp;


typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  channel;
} cmdGetTransceiverInfoReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  channel;
  uint32_t transceiverCapabilities;
  uint8_t  transceiverStatus;
  uint8_t  transceiverType;
  uint8_t  padding[2];
} cmdGetTransceiverInfoResp;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  transId;
  uint8_t  pad1;
  uint16_t rate;
  uint16_t pad2;
} cmdSetHeartbeatRateReq;

typedef struct {                // Could be 8 bytes but is 12 because the low bits
  uint8_t  cmdLen;              // of the time should be 4 byte aligned and found on
  uint8_t  cmdNo;               // the same place in all timestamped commands.
  uint16_t padding;
  uint16_t time[3];
  uint16_t padding2;
} cmdHeartbeatResp;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t channel;
  uint8_t bufNo;
  uint8_t rawMessage[14];
  uint8_t flags;
  uint8_t _padding0;
} cmdSetAutoTxBuffer;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  requestType;
  uint8_t  channel;             // For certain requests only
  uint32_t interval;            // D:o
  uint8_t  bufNo;               // D:o
  uint8_t  padding[3];
} cmdAutoTxBufferReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  responseType;
  uint8_t  bufferCount;
  uint32_t timerResolution;
  uint16_t capabilities;
  uint16_t padding0;
} cmdAutoTxBufferResp;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint16_t sofNr;
  uint16_t time[3];
  uint16_t padding;
} cmdTrefSofSeq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint16_t onOff;
} cmdSoftSyncOnOff;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint16_t throttle;
} cmdUsbThrottle;

#define THROTTLE_FLAG_READ                     1
#define THROTTLE_FLAG_WRITE                    2
typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint16_t throttle;
  uint32_t flag;
} cmdUsbThrottleScaled;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t subCmd;
  uint8_t reserved;
  uint8_t data[28];
} cmdGetExtendedInfoReq;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t subCmd;
  uint8_t status;
  uint8_t data[28];
} cmdGetExtendedInfoResp;

// Heartbeat via TCP for Iris
typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  seqNo;
  uint8_t  reserved;
  uint32_t time;
  uint8_t  data[16];
} cmdTcpKeepalive;


typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  channel;
  uint8_t  status;
  uint32_t interval;
} cmdTxInterval;


typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  channel;
  uint8_t  padding;
  uint16_t envvarLen;
  uint16_t padding2;
  uint32_t hash;
} cmdScriptEnvvarNotifyEvent;


typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t padding[2];
} cmdGetDeviceNameReq;

typedef struct {
  uint8_t cmdLen;
  uint8_t cmdNo;
  uint8_t index;
  uint8_t padding;
  uint8_t data[16];
} cmdGetDeviceNameResp;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  padding[2];
  uint32_t time_in_us;
} cmdPingReq, cmdPingResp;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  reserved[2];
  uint32_t code;
} cmdSetUnlockCode;


//Sub commands in CMD_GET_CAPABILITIES_REQ
#define CAP_SUB_CMD_DUMMY_NOT_IMPLEMENTED      0
#define CAP_SUB_CMD_DUMMY_UNAVAILABLE          1
#define CAP_SUB_CMD_SILENT_MODE                2
#define CAP_SUB_CMD_ERRFRAME                   3
#define CAP_SUB_CMD_BUS_STATS                  4
#define CAP_SUB_CMD_ERRCOUNT_READ              5
#define CAP_SUB_CMD_SINGLE_SHOT                6
#define CAP_SUB_CMD_SYNC_TX_FLUSH              7
#define CAP_SUB_CMD_HAS_LOGGER                 8
#define CAP_SUB_CMD_HAS_REMOTE                 9
#define CAP_SUB_CMD_HAS_SCRIPT                10

// the following are not capabilities/bits
#define CAP_SUB_CMD_DATA_START               1024
#define CAP_SUB_CMD_GET_LOGGER_INFO          CAP_SUB_CMD_DATA_START+1
#define CAP_SUB_CMD_REMOTE_INFO              CAP_SUB_CMD_DATA_START+2
#define CAP_SUB_CMD_HW_STATUS                CAP_SUB_CMD_DATA_START+3 // only used in hydra
#define CAP_SUB_CMD_FEATURE_EAN              CAP_SUB_CMD_DATA_START+4 // only used in hydra

// CAP_SUB_CMD_GET_LOGGER_TYPE
#define LOGGERTYPE_NOT_A_LOGGER 0
#define LOGGERTYPE_V1 1

// CAP_SUB_CMD_REMOTE_TYPE
#define REMOTE_TYPE_NOT_REMOTE  0
#define REMOTE_TYPE_WLAN 1
#define REMOTE_TYPE_LAN  2

typedef union {
  uint16_t padding;
  uint16_t channel;
} capExtraInfo_u;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint16_t pad;
  uint16_t subCmdNo;
  capExtraInfo_u  subData;
} cmdCapabilitiesReq;


typedef struct
{
  uint32_t mask;
  uint32_t value;
} channelCap32_t;


typedef struct
{
  uint32_t     webServer;
  uint32_t     remoteType;
} RemoteInfo_t;

typedef struct
{
  uint32_t data;
} Info_t;

//Status codes in CMD_GET_CAPABILITIES_RESP
#define CAP_STATUS_OK                          0
#define CAP_STATUS_NOT_IMPLEMENTED             1
#define CAP_STATUS_UNAVAILABLE                 2

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint16_t pad;
  uint16_t subCmdNo;
  uint16_t status;
  union {
    channelCap32_t silentMode;     // CAP_SUB_CMD_SILENT_MODE
    channelCap32_t errframeCap;    // CAP_SUB_CMD_ERRFRAME
    channelCap32_t busstatCap;     // CAP_SUB_CMD_BUS_STATS
    channelCap32_t errcountCap;    // CAP_SUB_CMD_ERRCOUNT_READ
    channelCap32_t syncTxFlushCap; // CAP_SUB_CMD_SYNC_TX_FLUSH
    channelCap32_t loggerCap;     // CAP_SUB_CMD_HAS_LOGGER
    channelCap32_t remoteCap;     // CAP_SUB_CMD_HAS_REMOTE
    channelCap32_t scriptCap;     // CAP_SUB_CMD_HAS_SCRIPT
    Info_t loggerType;            // CAP_SUB_CMD_GET_LOGGER_TYPE
    RemoteInfo_t remoteInfo;      // CAP_SUB_CMD_REMOTE_TYPE
  };
} cmdCapabilitiesResp;


#define SCRIPT_ENVVAR_SUBCMD_SET_START         1
#define SCRIPT_ENVVAR_SUBCMD_GET_START         2

#define SCRIPT_ENVVAR_RESP_OK                  0
#define SCRIPT_ENVVAR_RESP_UNKNOWN_VAR         1
#define SCRIPT_ENVVAR_RESP_WRONG_VAR_LEN       2
#define SCRIPT_ENVVAR_RESP_OUT_OF_MEMORY       3

// max 24 bytes
typedef union {
  struct {
    uint16_t bulkLen;
  } cmdStartSet;
  struct {
    uint16_t startOffset;
    uint16_t payloadLengthWanted;
  } cmdStartGet;
} commandData;

typedef struct {
  uint8_t     cmdLen;
  uint8_t     cmdNo;
  uint8_t     subCommand;       // start_get, start_set
  uint8_t     origin;
  uint32_t    hash;
  commandData subCmdData;
} cmdScriptEnvvarTransferCtrlReq;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  subCommand;          //return the same
  uint8_t  resp;
  uint32_t hash;
} cmdScriptEnvvarTransferCtrlResp;

typedef struct {
  uint8_t  cmdLen;
  uint8_t  cmdNo;
  uint8_t  origin;
  uint8_t  bulkDone;
  uint16_t offset;
  uint16_t length;
  uint32_t hash;
  uint8_t  bulkData[20];
} cmdScriptEnvvarTransferBulk;


typedef union {
  cmdHead                         head;

  cmdLogMessage                   logMessage;
  cmdLogTrig                      logTrig;
  cmdLogRtcTime                   logRtcTime;
  cmdRxCanMessage                 rxCanMessage;
  cmdTxCanMessage                 txCanMessage;
  cmdTxAck                        txAck;
  cmdTxRequest                    txRequest;
  cmdSetBusparamsReq              setBusparamsReq;
  cmdGetBusparamsReq              getBusparamsReq;
  cmdGetBusparamsResp             getBusparamsResp;
  cmdGetChipStateReq              getChipStateReq;
  cmdChipStateEvent               chipStateEvent;
  cmdSetDrivermodeReq             setDrivermodeReq;
  cmdGetDrivermodeReq             getDrivermodeReq;
  cmdGetDrivermodeResp            getDrivermodeResp;
  cmdResetChipReq                 resetChipReq;
  cmdResetCardReq                 resetCardReq;
  cmdStartChipReq                 startChipReq;
  cmdStartChipResp                startChipResp;
  cmdStopChipReq                  stopChipReq;
  cmdStopChipResp                 stopChipResp;
  cmdGetCardInfo2Req              getCardInfo2Req;
  cmdGetCardInfo2Resp             getCardInfo2Resp;
  cmdGetCardInfoReq               getCardInfoReq;
  cmdGetCardInfoResp              getCardInfoResp;
  cmdGetInterfaceInfoReq          getInterfaceInfoReq;
  cmdGetInterfaceInfoResp         getInterfaceInfoResp;
  cmdGetSoftwareInfoReq           getSoftwareInfoReq;
  cmdGetSoftwareInfoResp          getSoftwareInfoResp;
  cmdGetBusLoadReq                getBusLoadReq;
  cmdGetBusLoadResp               getBusLoadResp;
  cmdResetStatisticsReq           resetStatisticsReq;
  cmdErrorEvent                   errorEvent;
  cmdFlushQueue                   flushQueue;
  cmdFlushQueueResp               flushQueueResp;
  cmdNoCommand                    noCommand;
  cmdResetErrorCounter            resetErrorCounter;
  cmdCanErrorEvent                canErrorEvent;
  cmdCheckLicenseReq              checkLicenseReq;
  cmdCheckLicenseResp             checkLicenseResp;
  cmdReadClockReq                 readClockReq;
  cmdReadClockResp                readClockResp;
  cmdSelfTestReq                  selfTestReq;
  cmdSelfTestResp                 selfTestResp;
  cmdSetTransceiverModeReq        setTransceiverModeReq;
  cmdInternalDummy                internalDummy;

  cmdMemoCpldPrgReq               memoCpldPrgReq;
  cmdMemoCpldPrgResp              memoCpldPrgResp;

  cmdLedActionReq                 ledActionReq;
  cmdLedActionResp                ledActionResp;
  cmdMemoConfigModeReq            memoConfigModeReq;
  cmdMemoConfigModeResp           memoConfigModeResp;

  cmdSetIoPortsReq                setIoPortsReq;
  cmdGetIoPortsReq                getIoPortsReq;
  cmdGetIoPortsResp               getIoPortsResp;

  cmdGetTransceiverInfoReq        getTransceiverInfoReq;
  cmdGetTransceiverInfoResp       getTransceiverInfoResp;

  cmdSetHeartbeatRateReq          setHeartbeatRateReq;
  cmdHeartbeatResp                heartbeatResp;

  cmdSetAutoTxBuffer              setAutoTxBuffer;
  cmdAutoTxBufferReq              autoTxBufferReq;
  cmdAutoTxBufferResp             autoTxBufferResp;

  cmdTrefSofSeq                   trefSofSeq;
  cmdSoftSyncOnOff                softSyncOnOff;
  cmdUsbThrottle                  usbThrottle;
  cmdUsbThrottleScaled            usbThrottleScaled;
  cmdSound                        sound;

  cmdMemoGetDataReq               memoGetDataReq;
  cmdMemoGetDataResp              memoGetDataResp;
  cmdMemoPutDataStartReq          memoPutDataStartReq;
  cmdMemoPutDataReq               memoPutDataReq;
  cmdMemoPutDataResp              memoPutDataResp;
  cmdMemoAsyncopStartReq          memoAsyncopStartReq;
  cmdMemoAsyncopStartResp         memoAsyncopStartResp;
  cmdMemoAsyncopGetDataReq        memoAsyncopGetDataReq;
  cmdMemoAsyncopGetDataResp       memoAsyncopGetDataResp;
  cmdMemoAsyncopCancelReq         memoAsyncopCancelReq;
  cmdMemoAsyncopFinishedResp      memoAsyncopFinishedResp;
  cmdDiskFullInfo                 diskFullInfo;

  cmdReadUserParameter            readUserParameter;
  cmdWriteUserParameter           writeUserParameter;

  cmdGetExtendedInfoReq           getExtendedInfoReq;
  cmdGetExtendedInfoResp          getExtendedInfoResp;

  cmdTcpKeepalive                 tcpKeepalive;

  cmdTxInterval                   txInterval;

  cmdScriptCtrlReq                scriptCtrlReq;
  cmdScriptCtrlResp               scriptCtrlResp;

  cmdEnvvarCtrlReq                envvarCtrlReq;
  cmdEnvvarCtrlResp               envvarCtrlResp;

  cmdScriptEnvvarTransferCtrlReq  scriptEnvvarTransferCtrlReq;
  cmdScriptEnvvarTransferCtrlResp scriptEnvvarTransferCtrlResp;
  cmdScriptEnvvarTransferBulk     scriptEnvvarTransferBulk;
  cmdScriptEnvvarNotifyEvent      scriptEnvvarNotifyEvent;

  cmdGetDeviceNameReq             getDeviceNameReq;
  cmdGetDeviceNameResp            getDeviceNameResp;

  cmdPingReq                      pingReq;
  cmdPingResp                     pingResp;

  cmdSetUnlockCode                setUnlockCode;
  cmdCapabilitiesReq              capabilitiesReq;
  cmdCapabilitiesResp             capabilitiesResp;

#ifdef FILO_PRIVATE
  cmdFiloOtherCommand             o;
#endif
} filoCmd;


// Union for all *synchronized* responses from the card.
// The receive queue uses this union, so it is to be kept small.
typedef union {
  cmdHead           head;
  cmdLogMessage     logMessage;
  cmdLogTrig        logTrig;
  cmdLogRtcTime     logRtcTime;
  cmdRxCanMessage   rxCanMessage;
  cmdTxCanMessage   txCanMessage;
  cmdTxAck          txAck;
  cmdTxRequest      txRequest;
  cmdChipStateEvent chipStateEvent;
  cmdGetBusLoadResp getBusLoadResp;
  cmdStartChipResp  startChipResp;
  cmdStopChipResp   stopChipResp;
  cmdCanErrorEvent  canErrorEvent;
  cmdHeartbeatResp  heartbeatResp;
  cmdReadClockResp  readClockResp;
  cmdSelfTestResp   selfTestResp;
  cmdTrefSofSeq     trefSofSeq;
  cmdDiskFullInfo   diskFullInfo;
} filoResponse;

#pragma pack(pop)

CompilerAssert(sizeof(filoResponse) == 24);
// A basic sanity check of all structs:
#define FILO_CMD_ALIGNMENT(X) CompilerAssert((sizeof(X) % 4) == 0)
FILO_CMD_ALIGNMENT(cmdLogMessage                );
FILO_CMD_ALIGNMENT(cmdLogTrig                   );
FILO_CMD_ALIGNMENT(cmdLogRtcTime                );
FILO_CMD_ALIGNMENT(cmdRxCanMessage              );
FILO_CMD_ALIGNMENT(cmdTxCanMessage              );
FILO_CMD_ALIGNMENT(cmdTxAck                     );
FILO_CMD_ALIGNMENT(cmdTxRequest                 );
FILO_CMD_ALIGNMENT(cmdSetBusparamsReq           );
FILO_CMD_ALIGNMENT(cmdGetBusparamsReq           );
FILO_CMD_ALIGNMENT(cmdGetBusparamsResp          );
FILO_CMD_ALIGNMENT(cmdGetChipStateReq           );
FILO_CMD_ALIGNMENT(cmdChipStateEvent            );
FILO_CMD_ALIGNMENT(cmdSetDrivermodeReq          );
FILO_CMD_ALIGNMENT(cmdGetDrivermodeReq          );
FILO_CMD_ALIGNMENT(cmdGetDrivermodeResp         );
FILO_CMD_ALIGNMENT(cmdResetChipReq              );
FILO_CMD_ALIGNMENT(cmdResetCardReq              );
FILO_CMD_ALIGNMENT(cmdStartChipReq              );
FILO_CMD_ALIGNMENT(cmdStartChipResp             );
FILO_CMD_ALIGNMENT(cmdStopChipReq               );
FILO_CMD_ALIGNMENT(cmdStopChipResp              );
FILO_CMD_ALIGNMENT(cmdGetCardInfo2Req           );
FILO_CMD_ALIGNMENT(cmdGetCardInfo2Resp          );
FILO_CMD_ALIGNMENT(cmdGetCardInfoReq            );
FILO_CMD_ALIGNMENT(cmdGetCardInfoResp           );
FILO_CMD_ALIGNMENT(cmdGetInterfaceInfoReq       );
FILO_CMD_ALIGNMENT(cmdGetInterfaceInfoResp      );
FILO_CMD_ALIGNMENT(cmdGetSoftwareInfoReq        );
FILO_CMD_ALIGNMENT(cmdGetSoftwareInfoResp       );
FILO_CMD_ALIGNMENT(cmdGetBusLoadReq             );
FILO_CMD_ALIGNMENT(cmdGetBusLoadResp            );
FILO_CMD_ALIGNMENT(cmdResetStatisticsReq        );
FILO_CMD_ALIGNMENT(cmdErrorEvent                );
FILO_CMD_ALIGNMENT(cmdFlushQueue                );
FILO_CMD_ALIGNMENT(cmdFlushQueueResp            );
FILO_CMD_ALIGNMENT(cmdNoCommand                 );
FILO_CMD_ALIGNMENT(cmdResetErrorCounter         );
FILO_CMD_ALIGNMENT(cmdCanErrorEvent             );
FILO_CMD_ALIGNMENT(cmdCheckLicenseReq           );
FILO_CMD_ALIGNMENT(cmdCheckLicenseResp          );
FILO_CMD_ALIGNMENT(cmdReadClockReq              );
FILO_CMD_ALIGNMENT(cmdReadClockResp             );
FILO_CMD_ALIGNMENT(cmdSelfTestReq               );
FILO_CMD_ALIGNMENT(cmdSelfTestResp              );
FILO_CMD_ALIGNMENT(cmdSetTransceiverModeReq     );

FILO_CMD_ALIGNMENT(cmdMemoCpldPrgReq            );
FILO_CMD_ALIGNMENT(cmdMemoCpldPrgResp           );
FILO_CMD_ALIGNMENT(cmdLedActionReq              );
FILO_CMD_ALIGNMENT(cmdLedActionResp             );

FILO_CMD_ALIGNMENT(cmdMemoConfigModeReq         );
FILO_CMD_ALIGNMENT(cmdMemoConfigModeResp        );

FILO_CMD_ALIGNMENT(cmdSetIoPortsReq             );
FILO_CMD_ALIGNMENT(cmdGetIoPortsReq             );
FILO_CMD_ALIGNMENT(cmdGetIoPortsResp            );

FILO_CMD_ALIGNMENT(cmdGetTransceiverInfoReq     );
FILO_CMD_ALIGNMENT(cmdGetTransceiverInfoResp    );

FILO_CMD_ALIGNMENT(cmdSetHeartbeatRateReq       );
FILO_CMD_ALIGNMENT(cmdHeartbeatResp             );

FILO_CMD_ALIGNMENT(cmdSetAutoTxBuffer           );
FILO_CMD_ALIGNMENT(cmdAutoTxBufferReq           );
FILO_CMD_ALIGNMENT(cmdAutoTxBufferResp          );

FILO_CMD_ALIGNMENT(cmdTrefSofSeq                );
FILO_CMD_ALIGNMENT(cmdSoftSyncOnOff             );
FILO_CMD_ALIGNMENT(cmdUsbThrottle               );
FILO_CMD_ALIGNMENT(cmdUsbThrottleScaled         );

FILO_CMD_ALIGNMENT(cmdScriptEnvvarTransferCtrlReq       );
FILO_CMD_ALIGNMENT(cmdScriptEnvvarTransferCtrlResp       );
FILO_CMD_ALIGNMENT(cmdScriptEnvvarTransferBulk       );
FILO_CMD_ALIGNMENT(cmdScriptEnvvarNotifyEvent       );
FILO_CMD_ALIGNMENT(cmdEnvvarCtrlReq             );
FILO_CMD_ALIGNMENT(cmdEnvvarCtrlResp            );

FILO_CMD_ALIGNMENT(cmdGetDeviceNameReq          );
FILO_CMD_ALIGNMENT(cmdGetDeviceNameResp         );

FILO_CMD_ALIGNMENT(cmdPingReq                   );
FILO_CMD_ALIGNMENT(cmdPingResp                  );

FILO_CMD_ALIGNMENT(cmdSetUnlockCode             );

FILO_CMD_ALIGNMENT(cmdCapabilitiesReq           );
FILO_CMD_ALIGNMENT(cmdCapabilitiesResp          );


FILO_CMD_ALIGNMENT(cmdReadUserParameter         );
FILO_CMD_ALIGNMENT(cmdWriteUserParameter        );


CompilerAssert(sizeof(filoCmd) <= MAX_CMD_LEN);
CompilerAssert(sizeof(MemoDataFsInfo) == 24);
CompilerAssert(sizeof(MemoDataFsInfoB) == 24);
CompilerAssert(sizeof(MemoDataDiskInfoA) == 24);
CompilerAssert(sizeof(MemoDataDiskInfoB) == 24);
CompilerAssert(sizeof(MemoDataMiscInfo) == 24);
CompilerAssert(sizeof(MemoDataRtcInfo) <= 24);
CompilerAssert(sizeof(MemoInitDiskReq) == 24);
CompilerAssert(sizeof(MemoInitDiskResp) <= 24);

CompilerAssert(sizeof(cmdEnvvarCtrlReq) == 32);
CompilerAssert(sizeof(cmdEnvvarCtrlResp) == 32);

CompilerAssert(sizeof(cmdMemoGetDataReq) == 32);
CompilerAssert(sizeof(cmdMemoGetDataResp) == 32);
CompilerAssert(sizeof(cmdMemoPutDataStartReq) == 24);
CompilerAssert(sizeof(cmdMemoPutDataReq) == 32);
CompilerAssert(sizeof(cmdMemoPutDataResp) == 32);
CompilerAssert(sizeof(cmdMemoAsyncopStartReq) == 6);
CompilerAssert(sizeof(cmdMemoAsyncopStartResp) == 8);
CompilerAssert(sizeof(cmdMemoAsyncopFinishedResp) == 8);
CompilerAssert(sizeof(cmdMemoAsyncopGetDataReq) == 6);
CompilerAssert(sizeof(cmdMemoAsyncopGetDataResp) == 32);
CompilerAssert(sizeof(cmdMemoAsyncopCancelReq) == 6);

CompilerAssert(sizeof(cmdLogRtcTime) == 20);
CompilerAssert(sizeof(cmdRxCanMessage) == 24);
CompilerAssert(sizeof(cmdTxCanMessage) == 20);

CompilerAssert(sizeof(cmdHead) == 2);
CompilerAssert(sizeof(cmdLogMessage) == 24);
CompilerAssert(sizeof(cmdLogTrig) == 20);

CompilerAssert(sizeof(cmdTxAck) == 12);
CompilerAssert(sizeof(cmdTxRequest) == 12);
CompilerAssert(sizeof(cmdSetBusparamsReq) == 12);
CompilerAssert(sizeof(cmdGetBusparamsReq) == 4);
CompilerAssert(sizeof(cmdGetBusparamsResp) == 12);
CompilerAssert(sizeof(cmdGetChipStateReq) == 4);
CompilerAssert(sizeof(cmdChipStateEvent) == 16);
CompilerAssert(sizeof(cmdSetDrivermodeReq) == 8);
CompilerAssert(sizeof(cmdGetDrivermodeReq) == 4);
CompilerAssert(sizeof(cmdGetDrivermodeResp) == 8);
CompilerAssert(sizeof(cmdResetChipReq) == 4);
CompilerAssert(sizeof(cmdResetCardReq) == 4);
CompilerAssert(sizeof(cmdStartChipReq) == 4);
CompilerAssert(sizeof(cmdStartChipResp) == 4);
CompilerAssert(sizeof(cmdStopChipReq) == 4);
CompilerAssert(sizeof(cmdStopChipResp) == 4);
CompilerAssert(sizeof(cmdGetCardInfo2Req) == 4);
CompilerAssert(sizeof(cmdGetCardInfo2Resp) == 32);
CompilerAssert(sizeof(cmdGetCardInfoReq) == 4);
CompilerAssert(sizeof(cmdGetCardInfoResp) == 32);
CompilerAssert(sizeof(cmdGetInterfaceInfoReq) == 4);
CompilerAssert(sizeof(cmdGetInterfaceInfoResp) == 12);
CompilerAssert(sizeof(cmdGetSoftwareInfoReq) == 4);
CompilerAssert(sizeof(cmdGetSoftwareInfoResp) == 32);
CompilerAssert(sizeof(cmdGetBusLoadReq) == 4);
CompilerAssert(sizeof(cmdGetBusLoadResp) == 16);
CompilerAssert(sizeof(cmdResetStatisticsReq) == 4);
CompilerAssert(sizeof(cmdErrorEvent) == 16);
CompilerAssert(sizeof(cmdFlushQueue) == 8);
CompilerAssert(sizeof(cmdFlushQueueResp) == 8);
CompilerAssert(sizeof(cmdNoCommand) == 4);
CompilerAssert(sizeof(cmdResetErrorCounter) == 4);
CompilerAssert(sizeof(cmdCanErrorEvent) == 16);
CompilerAssert(sizeof(cmdCheckLicenseReq) == 4);
CompilerAssert(sizeof(cmdCheckLicenseResp) == 12);
CompilerAssert(sizeof(cmdReadClockReq) == 4);
CompilerAssert(sizeof(cmdReadClockResp) == 12);
CompilerAssert(sizeof(cmdSelfTestReq) == 4);
CompilerAssert(sizeof(cmdSelfTestResp) == 8);
CompilerAssert(sizeof(cmdSetTransceiverModeReq) == 8);
CompilerAssert(sizeof(cmdInternalDummy) == 4);

CompilerAssert(sizeof(cmdMemoCpldPrgReq) == 28);
CompilerAssert(sizeof(cmdMemoCpldPrgResp) == 4);

CompilerAssert(sizeof(cmdLedActionReq) == 8);
CompilerAssert(sizeof(cmdLedActionResp) == 4);
CompilerAssert(sizeof(cmdMemoConfigModeReq) == 12);
CompilerAssert(sizeof(cmdMemoConfigModeResp) == 16);

CompilerAssert(sizeof(cmdSetIoPortsReq) == 8);
CompilerAssert(sizeof(cmdGetIoPortsReq) == 4);
CompilerAssert(sizeof(cmdGetIoPortsResp) == 16);

CompilerAssert(sizeof(cmdGetTransceiverInfoReq) == 4);
CompilerAssert(sizeof(cmdGetTransceiverInfoResp) == 12);

CompilerAssert(sizeof(cmdSetHeartbeatRateReq) == 8);
CompilerAssert(sizeof(cmdHeartbeatResp) == 12);

CompilerAssert(sizeof(cmdSetAutoTxBuffer) == 20);
CompilerAssert(sizeof(cmdAutoTxBufferReq) == 12);
CompilerAssert(sizeof(cmdAutoTxBufferResp) == 12);

CompilerAssert(sizeof(cmdTrefSofSeq) == 12);
CompilerAssert(sizeof(cmdSoftSyncOnOff) == 4);
CompilerAssert(sizeof(cmdUsbThrottle) == 4);
CompilerAssert(sizeof(cmdUsbThrottleScaled) == 8);
CompilerAssert(sizeof(cmdSound) == 16);

CompilerAssert(sizeof(cmdDiskFullInfo) == 4);

CompilerAssert(sizeof(cmdReadUserParameter) == 16);
CompilerAssert(sizeof(cmdWriteUserParameter) == 24);

CompilerAssert(sizeof(cmdGetExtendedInfoReq) == 32);
CompilerAssert(sizeof(cmdGetExtendedInfoResp) == 32);

CompilerAssert(sizeof(cmdTcpKeepalive) == 24);

CompilerAssert(sizeof(cmdTxInterval) == 8);

CompilerAssert(sizeof(cmdScriptCtrlReq) == 28);
CompilerAssert(sizeof(cmdScriptCtrlResp) == 32);

CompilerAssert(sizeof(cmdEnvvarCtrlReq) == 32);
CompilerAssert(sizeof(cmdEnvvarCtrlResp) == 32);

CompilerAssert(sizeof(cmdScriptEnvvarTransferCtrlReq) == 12);
CompilerAssert(sizeof(cmdScriptEnvvarTransferCtrlResp) == 8);
CompilerAssert(sizeof(cmdScriptEnvvarTransferBulk) == 32);
CompilerAssert(sizeof(cmdScriptEnvvarNotifyEvent) == 12);

CompilerAssert(sizeof(cmdGetDeviceNameReq) == 4);
CompilerAssert(sizeof(cmdGetDeviceNameResp) == 20);

CompilerAssert(sizeof(cmdPingReq) == 8);
CompilerAssert(sizeof(cmdPingResp) == 8);

CompilerAssert(sizeof(cmdSetUnlockCode) == 8);
CompilerAssert(sizeof(cmdCapabilitiesReq) == 8);
CompilerAssert(sizeof(cmdCapabilitiesResp) == 16);
#undef FILO_CMD_ALIGNMENT

#endif //_FILO_CMDS_H_

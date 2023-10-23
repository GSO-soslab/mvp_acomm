#ifndef GOBY_ACOMMS_SEATRAC_CODEC_H
#define GOBY_ACOMMS_SEATRAC_CODEC_H


int RANGE_VALID         = 0b00000001;
int USBL_VALID          = 0b00000010;
int POSITION_VALID      = 0b00000100;
int POSITION_ENHANCED   = 0b00001000;
int POSITION_FLT_ERR    = 0b00010000;


enum ESeatracCmdId : uint8_t {
    ST_CID_INVALID = 0x00,
    // System Messages
    ST_CID_SYS_ALIVE = 0x01,                			/*!< Command sent to receive a simple alive message from the beacon. */
    ST_CID_SYS_INFO = 0x02,                 			/*!< Command sent to receive hardware & firmware identification information. */
    ST_CID_SYS_REBOOT = 0x03,               			/*!< Command sent to soft reboot the beacon. */
    ST_CID_SYS_ENGINEERING = 0x04,          			/*!< Command sent to perform engineering actions. */
    // Firmware Programming Messages
    ST_CID_PROG_INIT = 0x0D,                			/*!< Command sent to initialise a firmware programming sequence. */
    ST_CID_PROG_BLOCK = 0x0E,               			/*!< Command sent to transfer a firmware programming block. */
    ST_CID_PROG_UPDATE = 0x0F,              			/*!< Command sent to update the firmware once program transfer has completed. */
    // Status Messages
    ST_CID_STATUS = 0x10,                   			/*!< Command sent to request the current system status (AHRS, Depth, Temp, etc). */
    ST_CID_STATUS_CFG_GET = 0x11,           			/*!< Command sent to retrieve the configuration of the status system (message content and auto-output interval). */
    ST_CID_STATUS_CFG_SET = 0x12,           			/*!< Command sent to set the configuration of the status system (message content and auto-output interval). */
    // Settings Messages
    ST_CID_SETTINGS_GET = 0x15,             			/*!< Command sent to retrieve the working settings in use on the beacon. */
    ST_CID_SETTINGS_SET = 0x16,             			/*!< Command sent to set the working settings and apply them. They are NOT saved to permanent memory until ST_CID_ SETTINGS_SAVE is issued. The device will need to be rebooted after this to apply some of the changes. */
    ST_CID_SETTINGS_LOAD = 0x17,            			/*!< Command sent to load the working settings from permanent storage and apply them. Not all settings can be loaded and applied as they only affect the device on start-up. */
    ST_CID_SETTINGS_SAVE = 0x18,            			/*!< Command sent to save the working settings into permanent storage. */
    ST_CID_SETTINGS_RESET = 0x19,           			/*!< Command sent to restore the working settings to defaults, store them into permanent memory and apply them. */
    ST_CID_CONFIG_GET = 0x1A,							/*!< Command sent to receive a configuration message, containing Pressure Info and USBL calibration */
    // Calibration Messages
    ST_CID_CAL_ACTION = 0x20,               			/*!< Command sent to perform specific calibration actions. */
    ST_CID_CAL_AHRS_GET = 0x21,             			/*!< Command sent to retrieve the current AHRS calibration. */
    ST_CID_CAL_AHRS_SET = 0x22,             			/*!< Command sent to set the contents of the current AHRS calibration (and store to memory) */
    // Acoustic Transceiver Messages
    ST_CID_XCVR_ANALYSE = 0x30,             			/*!< Command sent to instruct the receiver to perform a noise analysis and report the results. */
    ST_CID_XCVR_TX_MSG = 0x31,              			/*!< Message sent when the transceiver transmits a message. */
    ST_CID_XCVR_RX_ERR = 0x32,              			/*!< Message sent when the transceiver receiver encounters an error. */
    ST_CID_XCVR_RX_MSG = 0x33,              			/*!< Message sent when the transceiver receives a message (not requiring a response). */
    ST_CID_XCVR_RX_REQ = 0x34,              			/*!< Message sent when the transceiver receives a request (requiring a response). */
    ST_CID_XCVR_RX_RESP = 0x35,             			/*!< Message sent when the transceiver receives a response (to a transmitted request). */
    ST_CID_XCVR_RX_UNHANDLED =	0x37,       			/*!< Message sent when a message has been received but not handled by the protocol stack. */
    ST_CID_XCVR_USBL =	0x38,               			/*!< Message sent when a USBL signal is decoded into an angular bearing. */
    ST_CID_XCVR_FIX =	0x39,               			/*!< Message sent when the transceiver gets a position/range fix on a beacon from a request/response. */
    ST_CID_XCVR_STATUS =	0x3A,           			/*!< Message sent to query the current transceiver state. */
    ST_CID_XCVR_TX_MSGCTRL_SET = 0x3B,					/*!< Message sent to set or query the Transmit Mode (BlockAll, BlockResponse, AllowAll) */
    // PING Protocol Messages
    ST_CID_PING_SEND = 0x40,                			/*!< Command sent to transmit a PING message. */
    ST_CID_PING_REQ = 0x41,                 			/*!< Message sent when a PING request is received. */
    ST_CID_PING_RESP = 0x42,                			/*!< Message sent when a PING response is received, or timeout occurs, with the echo response data. */
    ST_CID_PING_ERROR = 0x43,               			/*!< Message sent when a PING response error/timeout occurs. */
    // ECHO Protocol Messages
    ST_CID_ECHO_SEND = 0x48,                			/*!< Command sent to transmit an ECHO message. */
    ST_CID_ECHO_REQ = 0x49,                 			/*!< Message sent when an ECHO request is received. */
    ST_CID_ECHO_RESP = 0x4A,                			/*!< Message sent when an ECHO response is received, or timeout occurs, with the echo response data. */
    ST_CID_ECHO_ERROR = 0x4B,               			/*!< Message sent when an ECHO response error/timeout occurs. */
    // NAV Protocol Messages
    ST_CID_NAV_QUERY_SEND = 0x50,           			/*!< Message sent to query navigation information from a remote beacon. */
    ST_CID_NAV_QUERY_REQ = 0x51,            			/*!< Message sent from a beacon that receives a NAV_QUERY. */
    ST_CID_NAV_QUERY_RESP = 0x52,           			/*!< Message generated when the beacon received a response to a NAV_QUERY. */
    ST_CID_NAV_ERROR = 0x53,                			/*!< Message generated if there is a problem with a NAV_QUERY - i.e. timeout etc. */
    ST_CID_NAV_QUEUE_SET = 0x58,                        /*!< Message sent to queue a packet of data ready for remote interrogation */
    ST_CID_NAV_QUEUE_CLR = 0x59,                        /*!< Message sent to query the packet queue */
    ST_CID_NAV_QUEUE_STATUS = 0x5A,                     /*!< Message sent to query the status of queued packets for each beacon */
    ST_CID_NAV_STATUS_SEND = 0x5B,                      /*!< Message issued to broadcast status information to all other beacons */
    ST_CID_NAV_STATUS_RECEIVE = 0x5C,                   /*!< Message generated when a beacon receives a NAV_STATUS message */
    // DAT Protocol Messages
    ST_CID_DAT_SEND = 0x60,                 			/*!< Message sent to transmit a datagram to another beacon */
    ST_CID_DAT_RECEIVE = 0x61,              			/*!< Message generated when a beacon receives a datagram. */
    ST_CID_DAT_ERROR = 0x63,                			/*!< Message generated when a beacon response error/timeout occurs for ACKs. */
    ST_CID_DAT_QUEUE_SET = 0x64,            			/*!< Message sent to set the contents of the packet data queue. */
    ST_CID_DAT_QUEUE_CLR = 0x65,            			/*!< Message sent to clear the contents of the packet data queue. */
    ST_CID_DAT_QUEUE_STATUS = 0x66,         			/*!< Message sent to obtain the current status of the packet data queue. */
    //CFG Protocol Messages
    ST_CID_CFG_BEACON_GET = 0x80,						/*!< Message sent to read the current remote beacon transceiver setup*/
    ST_CID_CFG_BEACON_SET = 0x81,						/*!< Message sent to set new values for the remote beacom transceiver */
    ST_CID_CFG_BEACON_RESP = 0x82,						/*!< Message sent when a response of the configuration from the remote beacon is received back (as a MSG_OWAY type) */
};

enum AMSGTYPE
{
    MSG_OWAY,
    MSG_OWAYU,
    MSG_REQ,
    MSG_RESP,
    MSG_REQU,
    MSG_RESPU,
    MSG_REQX,
    MSG_RESPX,
    MSG_UNKNOWN = 0xFF
};

#pragma pack(push, 1)

struct HARDWARE_T
{
    uint16_t part_number;
    uint8_t part_rev;
    uint32_t serial_number;
    uint16_t flags_sys;
    uint16_t flags_user;
};

struct FIRMWARE_T
{
    bool valid;
    uint16_t part_number;
    uint8_t version_maj;
    uint8_t version_min;
    uint16_t version_build;
    uint32_t checksum;
};

struct PRESSURE_CAL_T
{
    uint32_t id;
    uint8_t type;
    int16_t pressure_min;
    int16_t pressure_max;
    uint8_t cal_day;
    uint8_t cal_month;
    uint16_t cal_year;
};

struct RANGE_T
{
    uint32_t range_count;
    int32_t range_time;
    uint16_t range_dist;
};

struct USBL_T
{
    uint8_t usbl_channels;
    int16_t usbl_rssi[4];
    int16_t usbl_azimuth;
    int16_t usbl_elevation;
    int16_t usbl_fit_error;
};

struct POSITION_T
{
    int16_t position_easting;
    int16_t position_northing;
    int16_t position_depth;
};

struct ACOFIX_STATIC_T
{
    uint8_t dest_id;
    uint8_t src_id;
    uint8_t flags;
    uint8_t msg_type;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    uint16_t depth;
    uint16_t vos;
    int16_t rssi;
};

struct ACOFIX_T
{
    ACOFIX_STATIC_T local;
    RANGE_T range;
    USBL_T usbl;
    POSITION_T position;
};

struct ACOMSG_T
{
    uint8_t msg_dest_id;
    uint8_t msg_src_id;
    uint8_t msg_type;
    uint16_t msg_depth;
    uint8_t msg_payload_id;
    uint8_t msg_payload_len;
    uint8_t msg[31];
};

struct CID_SYS_INFO
{
    uint32_t seconds;
    uint8_t section;
    HARDWARE_T hardware;
    FIRMWARE_T boot_firmware; 
    FIRMWARE_T main_firmware;
    uint8_t board_rev;
    uint8_t extended_info;
    uint8_t flags;
    uint8_t reserved[3];
    PRESSURE_CAL_T pressure_sensor;
};

struct CID_STATUS
{
    uint8_t status_bits;
    uint64_t timestamp;
    uint16_t env_supply;
    int16_t env_temp;
    int32_t env_pressure;
    int32_t env_depth;
    uint16_t env_vos;
    int16_t att_yaw;
    int16_t att_pitch;
    int16_t att_roll;
    uint8_t mag_cal_buf;
    bool mag_cal_valid;
    uint32_t mag_cal_age;
    uint8_t mag_cal_fit;
    int16_t acc_lim_min_x;
    int16_t acc_lim_min_y;
    int16_t acc_lim_min_z;
    int16_t acc_lim_max_x;
    int16_t acc_lim_max_y;
    int16_t acc_lim_max_z;
    int16_t ahrs_raw_acc_x;
    int16_t ahrs_raw_acc_y;
    int16_t ahrs_raw_acc_z;
    int16_t ahrs_raw_mag_x;
    int16_t ahrs_raw_mag_y;
    int16_t ahrs_raw_mag_z;
    int16_t ahrs_raw_gyro_x;
    int16_t ahrs_raw_gyro_y;
    int16_t ahrs_raw_gyro_z;
    int16_t ahrs_comp_acc_x;
    int16_t ahrs_comp_acc_y;
    int16_t ahrs_comp_acc_z;
    int16_t ahrs_comp_mag_x;
    int16_t ahrs_comp_mag_y;
    int16_t ahrs_comp_mag_z;
    int16_t ahrs_comp_gyro_x;
    int16_t ahrs_comp_gyro_y;
    int16_t ahrs_comp_gyro_z;


};

struct CID_XCVR_RX
{
    ACOFIX_T aco_fix;
    uint8_t msg_dest_id;
    uint8_t msg_src_id;
    uint8_t msg_type;
    uint16_t msg_depth;
    uint8_t msg_payload_id;
    uint8_t msg_payload_len;
    uint8_t msg[31];
};

struct CID_DAT_RX
{
    ACOFIX_T aco_fix;
    bool ack_flag;
    uint8_t packet_len;
    uint8_t packet_data[31];
    bool local_flag;
};

struct CID_DAT_SEND
{
    uint8_t cid_e = 0x60;
    uint8_t dest_id;
    uint8_t amsgtype;
    uint8_t packet_len;
};

#pragma(pop)

#endif
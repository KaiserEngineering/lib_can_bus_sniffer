/* Author : Matthew Kaiser */
#ifndef LIB_CAN_BUS_SNIFFER_H_
#define LIB_CAN_BUS_SNIFFER_H_

#include <stdio.h>
#include <stdint.h>
#include "lib_pid.h"

#define NUM_CAN_SNIFF 25

#define CAN_SNIFF_MAX_DEFINITIONS NUM_CAN_SNIFF

typedef void (*CAN_SNIFF_FILTER)( uint16_t id, uint8_t enable );

typedef enum _pid_supported_status {
    PID_NOT_SUPPORTED,
    PID_SUPPORTED
} PID_SUPPORTED_STATUS, *PPID_SUPPORTED_STATUS;

typedef enum _can_sniff_json_status {
    CAN_SNIFF_JSON_ERROR,
    CAN_SNIFF_JSON_OK
} CAN_SNIFF_JSON_STATUS, *PCAN_SNIFF_JSON_STATUS;

typedef enum _can_sniff_byte_order {
    CAN_SNIFF_BYTE_ORDER_MSB0,
    CAN_SNIFF_BYTE_ORDER_LSB0
} CAN_SNIFF_BYTE_ORDER, *PCAN_SNIFF_BYTE_ORDER;

/* Runtime form of one JSON sniffer definition. This is the small subset needed
 * while packets are flowing: which PID to update, which CAN ID to listen to,
 * how to extract the raw signal, and how to scale it into engineering units. */
typedef struct _can_sniff_signal_definition {
    uint32_t pid_uuid;
    uint16_t arbitration_id;
    uint16_t start_bit;
    uint8_t length;
    CAN_SNIFF_BYTE_ORDER byte_order;
    uint8_t is_signed;
    float scale;
    float offset;
    uint8_t has_invalid_raw;
    uint32_t invalid_raw;
} CAN_SNIFF_SIGNAL_DEFINITION, *PCAN_SNIFF_SIGNAL_DEFINITION;

typedef struct _can_sniffer_packet_manager {
	uint8_t status;
		#define CAN_SNIFF_INIT 0x0001

	/* Number of PIDs being streamed */
    uint8_t num_pids;

    /* Callback to add a CAN bus filter */
    CAN_SNIFF_FILTER filter;

    PTR_PID_DATA stream[NUM_CAN_SNIFF];

    /* Parallel to stream[]. Caching the definition avoids searching the loaded
     * definition table for every received CAN packet. */
    PCAN_SNIFF_SIGNAL_DEFINITION stream_definition[NUM_CAN_SNIFF];

    /* Populated from CAN_Sniffer_Load_JSON(). This table answers support checks,
     * filter setup, and decode rules. */
    CAN_SNIFF_SIGNAL_DEFINITION definition[CAN_SNIFF_MAX_DEFINITIONS];

    uint8_t num_definitions;

} CAN_SNIFFER_PACKET_MANAGER, *PCAN_SNIFFER_PACKET_MANAGER;

CAN_SNIFF_JSON_STATUS CAN_Sniffer_Load_JSON( PCAN_SNIFFER_PACKET_MANAGER dev, const char *json );
PID_SUPPORTED_STATUS CAN_Sniffer_PID_Supported( PCAN_SNIFFER_PACKET_MANAGER dev, PTR_PID_DATA pid );
PID_SUPPORTED_STATUS CAN_Sniffer_Add_PID( PCAN_SNIFFER_PACKET_MANAGER dev, PTR_PID_DATA pid );
PID_SUPPORTED_STATUS CAN_Sniffer_Remove_PID( PCAN_SNIFFER_PACKET_MANAGER dev, PTR_PID_DATA pid );
void CAN_Sniffer_Add_Packet( PCAN_SNIFFER_PACKET_MANAGER dev, uint16_t arbitration_id, uint8_t* packet_data );
void CAN_Sniffer_Initialize( PCAN_SNIFFER_PACKET_MANAGER dev );
void CAN_Sniffer_tick( void );



#endif // LIB_CAN_BUS_SNIFF_H

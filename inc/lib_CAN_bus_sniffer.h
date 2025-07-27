/* Author : Matthew Kaiser */
#ifndef LIB_CAN_BUS_SNIFFER_H_
#define LIB_CAN_BUS_SNIFFER_H_

#include <stdio.h>
#include <stdint.h>
#include "lib_pid.h"
#include "lib_unit_conversion.h"

#define NUM_CAN_SNIFF 25

#define SNIFF_TRANS_ACTUAL_GEAR_ID     0x70
#define SNIFF_ACCEL_PEDAL_POS_ID       0x080
#define SNIFF_ENGINE_RPM_ID            0x090
#define SNIFF_BOOST_PRESSURE_ID        0xF8
#define SNIFF_ENGINE_OIL_TEMP_ID       0xF8
#define SNIFF_GAUGE_BRIGHTNESS_ID      0xC8
#define SNIFF_VEHICLE_SPEED_ID         0x130
#define SNIFF_BRAKE_PEDAL_STATUS_ID    0x080
#define SNIFF_EMERGENCY_BRAKE_STATUS_ID    0x0C8
#define SNIFF_REVERSE_STATUS_ID            0x080
#define SNIFF_CRUISE_CONTROL_ON_BUTTON_ID  0x030
#define SNIFF_CRUISE_CONTROL_OFF_BUTTON_ID 0x030
#define SNIFF_CRUISE_CONTROL_SET_PLUS_BUTTON_ID 0x030
#define SNIFF_CRUISE_CONTROL_SET_MINUS_BUTTON_ID 0x030
#define SNIFF_CRUISE_CONTROL_RES_BUTTON_ID 0x030
#define SNIFF_CRUISE_CONTROL_CAN_BUTTON_ID 0x030
#define SNIFF_LATERAL_ACCELERATION_ID 0x180
#define SNIFF_LONGITUDINAL_ACCELERATION_ID 0x160

typedef void (*CAN_SNIFF_FILTER)( uint16_t id, uint8_t enable );

typedef enum _pid_supported_status {
    PID_NOT_SUPPORTED,
    PID_SUPPORTED
} PID_SUPPORTED_STATUS, *PPID_SUPPORTED_STATUS;

typedef struct _can_sniffer_packet_manager {
	uint8_t status;
		#define CAN_SNIFF_INIT 0x0001

	/* Number of PIDs being streamed */
    uint8_t num_pids;

    /* Callback to add a CAN bus filter */
    CAN_SNIFF_FILTER filter;

    PTR_PID_DATA stream[NUM_CAN_SNIFF];

} CAN_SNIFFER_PACKET_MANAGER, *PCAN_SNIFFER_PACKET_MANAGER;

PID_SUPPORTED_STATUS CAN_Sniffer_PID_Supported( PTR_PID_DATA pid );
PID_SUPPORTED_STATUS CAN_Sniffer_Add_PID( PCAN_SNIFFER_PACKET_MANAGER dev, PTR_PID_DATA pid );
PID_SUPPORTED_STATUS CAN_Sniffer_Remove_PID( PCAN_SNIFFER_PACKET_MANAGER dev, PTR_PID_DATA pid );
void CAN_Sniffer_Add_Packet( PCAN_SNIFFER_PACKET_MANAGER dev, uint16_t arbitration_id, uint8_t* packet_data );
void CAN_Sniffer_Initialize( PCAN_SNIFFER_PACKET_MANAGER dev );
void CAN_Sniffer_tick( void );



#endif // LIB_CAN_BUS_SNIFF_H

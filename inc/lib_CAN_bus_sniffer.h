/* Author : Matthew Kaiser */
#ifndef LIB_CAN_BUS_SNIFFER_H_
#define LIB_CAN_BUS_SNIFFER_H_

#include "stdio.h"
#include "lib_pid.h"
#include "lib_unit_conversion.h"

#define NUM_CAN_SNIFF 25

#ifdef FORD_FOCUS_STRS_2013_2018

    #ifdef MODE1_TRANS_ACTUAL_GEAR_SUPPORTED
    #define SNIFF_TRANS_ACTUAL_GEAR_PID    MODE1_TRANS_ACTUAL_GEAR
    #define SNIFF_TRANS_ACTUAL_GEAR_ID     0x70
    #endif

    #ifdef MODE1_REL_ACCELERATOR_PEDAL_POS_SUPPORTED
    #define SNIFF_ACCEL_PEDAL_POS_PID      MODE1_REL_ACCELERATOR_PEDAL_POS
    #define SNIFF_ACCEL_PEDAL_POS_ID       0x080
    #endif

    #ifdef MODE1_ENGINE_RPM_SUPPORTED
    #define SNIFF_ENGINE_RPM_PID           MODE1_ENGINE_RPM
    #define SNIFF_ENGINE_RPM_ID            0x090
    #endif

    #ifdef MODE1_TURBO_INLET_PRESSURE_SUPPORTED
    #define SNIFF_BOOST_PRESSURE_PID       MODE1_TURBO_INLET_PRESSURE
    #define SNIFF_BOOST_PRESSURE_ID        0xF8
    #endif

    #ifdef MODE1_ENGINE_OIL_TEMPERATURE_SUPPORTED
    #define SNIFF_ENGINE_OIL_TEMP_PID      MODE1_ENGINE_OIL_TEMPERATURE
    #define SNIFF_ENGINE_OIL_TEMP_ID       0xF8
    #endif

    #ifdef SNIFF_GAUGE_BRIGHTNESS_SUPPORTED
    #define SNIFF_GAUGE_BRIGHTNESS_PID     SNIFF_GAUGE_BRIGHTNESS
    #define SNIFF_GAUGE_BRIGHTNESS_ID      0xC8
    #endif

    #ifdef MODE1_VEHICLE_SPEED_SUPPORTED
    #define SNIFF_VEHICLE_SPEED_PID        MODE1_VEHICLE_SPEED
    #define SNIFF_VEHICLE_SPEED_ID         0x130
    #endif

#endif

typedef void (*CAN_SNIFF_FLAG_UPDATE)( uint16_t flag, uint8_t bit );
typedef void (*CAN_SNIFF_FILTER)( uint16_t id );

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
PID_SUPPORTED_STATUS CAN_Sniffer_Add_PID(  PCAN_SNIFFER_PACKET_MANAGER dev, PTR_PID_DATA pid );
void CAN_Sniffer_Add_Packet( PCAN_SNIFFER_PACKET_MANAGER dev, uint16_t arbitration_id, uint8_t* packet_data );
void CAN_Sniffer_Initialize( PCAN_SNIFFER_PACKET_MANAGER dev );
void CAN_Sniffer_tick( void );



#endif // LIB_CAN_BUS_SNIFF_H

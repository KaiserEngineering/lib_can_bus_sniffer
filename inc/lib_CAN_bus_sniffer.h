/* Author : Matthew Kaiser */
#ifndef LIB_CAN_BUS_SNIFFER_H_
#define LIB_CAN_BUS_SNIFFER_H_

#include "stdio.h"
#include "lib_pid.h"
#include "lib_unit_conversion.h"

#define NUM_CAN_SNIFF 25

#ifdef FORD_FOCUS_STRS_2013_2018

    #if defined(MODE1_TRANS_ACTUAL_GEAR_SUPPORTED) || !defined(LIMIT_PIDS)
    #define SNIFF_TRANS_ACTUAL_GEAR_PID    MODE1_TRANS_ACTUAL_GEAR
    #define SNIFF_TRANS_ACTUAL_GEAR_ID     0x70
    #endif

    #if defined(MODE1_RELATIVE_ACCELERATOR_PEDAL_POSITION_SUPPORTED) || !defined(LIMIT_PIDS)
    #define SNIFF_ACCEL_PEDAL_POS_PID      MODE1_RELATIVE_ACCELERATOR_PEDAL_POSITION
    #define SNIFF_ACCEL_PEDAL_POS_ID       0x080
    #endif

    #if defined(MODE1_ENGINE_SPEED_SUPPORTED) || !defined(LIMIT_PIDS)
    #define SNIFF_ENGINE_RPM_PID           MODE1_ENGINE_SPEED
    #define SNIFF_ENGINE_RPM_ID            0x090
    #endif

    #if defined(MODE1_TURBOCHARGER_COMPRESSOR_INLET_PRESSURE_SUPPORTED) || !defined(LIMIT_PIDS)
    #define SNIFF_BOOST_PRESSURE_PID       MODE1_TURBOCHARGER_COMPRESSOR_INLET_PRESSURE
    #define SNIFF_BOOST_PRESSURE_ID        0xF8
    #endif

    #if defined(MODE1_ENGINE_OIL_TEMPERATURE_SUPPORTED) || !defined(LIMIT_PIDS)
    #define SNIFF_ENGINE_OIL_TEMP_PID      MODE1_ENGINE_OIL_TEMPERATURE
    #define SNIFF_ENGINE_OIL_TEMP_ID       0xF8
    #endif

    #if defined(SNIFF_GAUGE_BRIGHTNESS_SUPPORTED) || !defined(LIMIT_PIDS)
    #define SNIFF_GAUGE_BRIGHTNESS_PID     SNIFF_GAUGE_BRIGHTNESS
    #define SNIFF_GAUGE_BRIGHTNESS_ID      0xC8
    #endif

    #if defined(MODE1_VEHICLE_SPEED_SUPPORTED) || !defined(LIMIT_PIDS)
    #define SNIFF_VEHICLE_SPEED_PID        MODE1_VEHICLE_SPEED
    #define SNIFF_VEHICLE_SPEED_ID         0x130
    #endif

    #if defined(SNIFF_BRAKE_PEDAL_STATUS_SUPPORTED) || !defined(LIMIT_PIDS)
    #define SNIFF_BRAKE_PEDAL_STATUS_PID   SNIFF_BRAKE_PEDAL_STATUS
    #define SNIFF_BRAKE_PEDAL_STATUS_ID    0x080
    #endif

    #if defined(SNIFF_EMERGENCY_BRAKE_STATUS_SUPPORTED) || !defined(LIMIT_PIDS)
    #define SNIFF_EMERGENCY_BRAKE_STATUS_PID   SNIFF_EMERGENCY_BRAKE_STATUS
    #define SNIFF_EMERGENCY_BRAKE_STATUS_ID    0x0C8
    #endif

    #if defined(SNIFF_REVERSE_STATUS_SUPPORTED) || !defined(LIMIT_PIDS)
    #define SNIFF_REVERSE_STATUS_PID           SNIFF_REVERSE_STATUS
    #define SNIFF_REVERSE_STATUS_ID            0x080
    #endif

    #if defined(SNIFF_CRUISE_CONTROL_ON_BUTTON_SUPPORTED) || !defined(LIMIT_PIDS)
    #define SNIFF_CRUISE_CONTROL_ON_BUTTON_PID SNIFF_CRUISE_CONTROL_ON_BUTTON
    #define SNIFF_CRUISE_CONTROL_ON_BUTTON_ID  0x030
    #endif

    #if defined(SNIFF_CRUISE_CONTROL_OFF_BUTTON_SUPPORTED) || !defined(LIMIT_PIDS)
    #define SNIFF_CRUISE_CONTROL_OFF_BUTTON_PID SNIFF_CRUISE_CONTROL_OFF_BUTTON
    #define SNIFF_CRUISE_CONTROL_OFF_BUTTON_ID 0x030
    #endif

    #if defined(SNIFF_CRUISE_CONTROL_SET_PLUS_BUTTON_SUPPORTED) || !defined(LIMIT_PIDS)
    #define SNIFF_CRUISE_CONTROL_SET_PLUS_BUTTON_PID SNIFF_CRUISE_CONTROL_SET_PLUS_BUTTON
    #define SNIFF_CRUISE_CONTROL_SET_PLUS_BUTTON_ID 0x030
    #endif

    #if defined(SNIFF_CRUISE_CONTROL_SET_MINUS_BUTTON_SUPPORTED) || !defined(LIMIT_PIDS)
    #define SNIFF_CRUISE_CONTROL_SET_MINUS_BUTTON_PID SNIFF_CRUISE_CONTROL_SET_MINUS_BUTTON
    #define SNIFF_CRUISE_CONTROL_SET_MINUS_BUTTON_ID 0x030
    #endif

    #if defined(SNIFF_CRUISE_CONTROL_RES_BUTTON_SUPPORTED) || !defined(LIMIT_PIDS)
    #define SNIFF_CRUISE_CONTROL_RES_BUTTON_PID SNIFF_CRUISE_CONTROL_RES_BUTTON
    #define SNIFF_CRUISE_CONTROL_RES_BUTTON_ID 0x030
    #endif

    #if defined(SNIFF_CRUISE_CONTROL_CAN_BUTTON_SUPPORTED) || !defined(LIMIT_PIDS)
    #define SNIFF_CRUISE_CONTROL_CAN_BUTTON_PID SNIFF_CRUISE_CONTROL_CAN_BUTTON
    #define SNIFF_CRUISE_CONTROL_CAN_BUTTON_ID 0x030
    #endif

    #if defined(SNIFF_LATERAL_ACCELERATION_SUPPORTED) || !defined(LIMIT_PIDS)
    #define SNIFF_LATERAL_ACCELERATION_PID SNIFF_LATERAL_ACCELERATION
    #define SNIFF_LATERAL_ACCELERATION_ID 0x180
    #endif

    #if defined(SNIFF_LONGITUDINAL_ACCELERATION_SUPPORTED) || !defined(LIMIT_PIDS)
    #define SNIFF_LONGITUDINAL_ACCELERATION_PID SNIFF_LONGITUDINAL_ACCELERATION
    #define SNIFF_LONGITUDINAL_ACCELERATION_ID 0x160
    #endif


#endif

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
PID_SUPPORTED_STATUS CAN_Sniffer_Add_PID( PCAN_SNIFFER_PACKET_MANAGER dev, PTR_PID_DATA pid );
PID_SUPPORTED_STATUS CAN_Sniffer_Remove_PID( PCAN_SNIFFER_PACKET_MANAGER dev, PTR_PID_DATA pid );
void CAN_Sniffer_Add_Packet( PCAN_SNIFFER_PACKET_MANAGER dev, uint16_t arbitration_id, uint8_t* packet_data );
void CAN_Sniffer_Initialize( PCAN_SNIFFER_PACKET_MANAGER dev );
void CAN_Sniffer_tick( void );



#endif // LIB_CAN_BUS_SNIFF_H

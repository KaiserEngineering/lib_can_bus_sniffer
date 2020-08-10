/* Author : Matthew Kaiser */
#ifndef LIB_CAN_BUS_DECODE_H_
#define LIB_CAN_BUS_DECODE_H_

#include "stdio.h"
#include "lib_pid.h"

#define NUM_CAN_DECODE 25

#define DECODE_ENGINE_RPM_PID 0x0C
#define DECODE_ENGINE_RRPM_ID 0x090

#define DECODE_ACCEL_PEDAL_POS_PID  0xC5
#define DECODE_ACCEL_PEDAL_POS_ID   0x080

#define DECODE_BOOST_PRESSURE_PID   0x6F
#define DECODE_BOOST_PRESSURE_ID    0xF8

#define DECODE_ENGINE_OIL_TEMP_PID 0x5C
#define DECODE_ENGINE_OIL_TEMP_ID  0xF8

typedef void (*CAN_DECODE_FLAG_UPDATE)( uint16_t flag, uint8_t bit );
typedef void (*CAN_DECODE_FILTER)( uint16_t id );

typedef enum _can_bus_decode {
    CAN_DECODE_IDLE,
    CAN_DECODE_NEW_DATA
} CAN_BUS_DECODE, *PCAN_BUS_DECODE;

typedef enum _pid_supported_status {
    PID_NOT_SUPPORTED,
    PID_SUPPORTED
} PID_SUPPORTED_STATUS, *PPID_SUPPORTED_STATUS;

typedef struct _can_decode_packet_manager {
	uint8_t status;
		#define CAN_DECODE_INIT 0x0001

	/* Number of PIDs being streamed */
    uint8_t num_pids;

    /* PID requested */
    uint16_t pid_request[NUM_CAN_DECODE];

    /* Float value of the PID */
    float pid_results[NUM_CAN_DECODE];

    /* Callback to add a CAN bus filter */
    CAN_DECODE_FILTER filter;

    PTR_PID_DATA stream[NUM_CAN_DECODE];

} CAN_DECODE_PACKET_MANAGER, *PCAN_DECODE_PACKET_MANAGER;

PID_SUPPORTED_STATUS CAN_Decode_Supported( PTR_PID_DATA pid );
PID_SUPPORTED_STATUS CAN_Decode_Add_PID(  PCAN_DECODE_PACKET_MANAGER dev, PTR_PID_DATA pid );
float CAN_Decode_Get_Value_Byte_PID( PCAN_DECODE_PACKET_MANAGER dev, uint16_t pid );
void CAN_Decode_Add_Packet( PCAN_DECODE_PACKET_MANAGER dev, uint16_t arbitration_id, uint8_t* packet_data );
void CAN_Decode_Initialize( PCAN_DECODE_PACKET_MANAGER dev );



#endif // LIB_CAN_BUS_DECODE_H

#include "unity.h"
#include "lib_CAN_bus_sniffer.h"
#include "stdio.h"
#include "lib_pid.h"
#include "lib_unit_conversion.h"

CAN_SNIFFER_PACKET_MANAGER sniffer;

uint16_t filter_id = 0xFFFF;

void filter ( uint16_t id )
{
    filter_id = id;
}

void setUp( void )
{
    filter_id = 0xFFFF;

    /* lib_can_bus_sniffer initialization */
    sniffer.filter = filter;

    CAN_Sniffer_Initialize(&sniffer);
}

void tearDown( void )
{
    filter_id = 0xFFFF;
}

void test_add_oil_temp_stream(void)
{
    PID_SUPPORTED_STATUS status = 0xFF;
    PID_DATA pid_req = { .pid = MODE1_ENGINE_OIL_TEMPERATURE, .mode = MODE1, .pid_unit = PID_UNITS_CELCIUS };
    status = CAN_Sniffer_Add_PID( &sniffer, &pid_req );
    TEST_ASSERT_EQUAL( PID_SUPPORTED , status );
    TEST_ASSERT_EQUAL( MODE1_ENGINE_OIL_TEMPERATURE , sniffer.stream[0]->pid );
    TEST_ASSERT_EQUAL( MODE1 , sniffer.stream[0]->mode );
}

void test_add_turbo_inlet_pressure_stream(void)
{
    PID_SUPPORTED_STATUS status = 0xFF;
    PID_DATA pid_req = { .pid = MODE1_TURBO_INLET_PRESSURE, .mode = MODE1, .pid_unit = PID_UNITS_KPA };
    status = CAN_Sniffer_Add_PID( &sniffer, &pid_req );
    TEST_ASSERT_EQUAL( PID_SUPPORTED , status );
    TEST_ASSERT_EQUAL( MODE1_TURBO_INLET_PRESSURE , sniffer.stream[0]->pid );
    TEST_ASSERT_EQUAL( MODE1 , sniffer.stream[0]->mode );
}

void test_add_multiple_to_stream(void)
{
    PID_SUPPORTED_STATUS status = 0xFF;
    PID_DATA pid_req = { .pid = MODE1_ENGINE_OIL_TEMPERATURE, .mode = MODE1, .pid_unit = PID_UNITS_CELCIUS };
    status = CAN_Sniffer_Add_PID( &sniffer, &pid_req );
    TEST_ASSERT_EQUAL( PID_SUPPORTED , status );
    TEST_ASSERT_EQUAL( MODE1_ENGINE_OIL_TEMPERATURE , sniffer.stream[0]->pid );
    TEST_ASSERT_EQUAL( MODE1 , sniffer.stream[0]->mode );

    status = 0xFF;
    pid_req.pid = MODE1_TURBO_INLET_PRESSURE;
    pid_req.mode = MODE1;
    pid_req.pid_unit = PID_UNITS_KPA;
    status = CAN_Sniffer_Add_PID( &sniffer, &pid_req );
    TEST_ASSERT_EQUAL( PID_SUPPORTED , status );
    TEST_ASSERT_EQUAL( MODE1_TURBO_INLET_PRESSURE , sniffer.stream[1]->pid );
    TEST_ASSERT_EQUAL( MODE1 , sniffer.stream[1]->mode );
}
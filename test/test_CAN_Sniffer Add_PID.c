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
    PID_SUPPORTED_STATUS status;
    PID_DATA stream[NUM_CAN_SNIFF];
    status = 0xFF;
    
    /* Parameter 0 = Engine Oil Temperature */
    stream[0].pid      = MODE1_ENGINE_OIL_TEMPERATURE;
    stream[0].mode     = MODE1;
    stream[0].pid_unit = PID_UNITS_CELCIUS;

    /* Add Parameter 0 */
    status = CAN_Sniffer_Add_PID( &sniffer, &stream[0] );

    /* Verify it was added to the correct slot */
    TEST_ASSERT_EQUAL( PID_SUPPORTED , status );
    TEST_ASSERT_EQUAL( MODE1_ENGINE_OIL_TEMPERATURE , sniffer.stream[0]->pid );
    TEST_ASSERT_EQUAL( MODE1 , sniffer.stream[0]->mode );
}

void test_add_turbo_inlet_pressure_stream(void)
{
    PID_SUPPORTED_STATUS status;
    PID_DATA stream[NUM_CAN_SNIFF];
    status = 0xFF;
    
    /* Parameter 0 = Turbo inlet pressure */
    stream[0].pid      = MODE1_TURBOCHARGER_COMPRESSOR_INLET_PRESSURE;
    stream[0].mode     = MODE1;
    stream[0].pid_unit = PID_UNITS_CELCIUS;

    /* Add Parameter 0 */
    status = CAN_Sniffer_Add_PID( &sniffer, &stream[0] );

    /* Verify it was added to the correct slot */
    TEST_ASSERT_EQUAL( PID_SUPPORTED , status );
    TEST_ASSERT_EQUAL( MODE1_TURBOCHARGER_COMPRESSOR_INLET_PRESSURE , sniffer.stream[0]->pid );
    TEST_ASSERT_EQUAL( MODE1 , sniffer.stream[0]->mode );
}

void test_add_multiple_to_stream(void)
{
    PID_SUPPORTED_STATUS status;
    PID_DATA stream[NUM_CAN_SNIFF];
    status = 0xFF;

    /* Parameter 0 = Engine Oil Temperature */
    stream[0].pid      = MODE1_ENGINE_OIL_TEMPERATURE;
    stream[0].mode     = MODE1;
    stream[0].pid_unit = PID_UNITS_CELCIUS;

    /* Parameter 1 = Turbo inlet pressure */
    stream[1].pid      = MODE1_TURBOCHARGER_COMPRESSOR_INLET_PRESSURE;
    stream[1].mode     = MODE1;
    stream[1].pid_unit = PID_UNITS_CELCIUS;

    status = CAN_Sniffer_Add_PID( &sniffer, &stream[0] );
    TEST_ASSERT_EQUAL( PID_SUPPORTED , status );
    TEST_ASSERT_EQUAL( MODE1_ENGINE_OIL_TEMPERATURE , sniffer.stream[0]->pid );
    TEST_ASSERT_EQUAL( MODE1 , sniffer.stream[0]->mode );

    status = 0xFF;
    status = CAN_Sniffer_Add_PID( &sniffer, &stream[1] );

    TEST_ASSERT_EQUAL( PID_SUPPORTED , status );
    TEST_ASSERT_EQUAL( MODE1_TURBOCHARGER_COMPRESSOR_INLET_PRESSURE , sniffer.stream[1]->pid );
    TEST_ASSERT_EQUAL( MODE1 , sniffer.stream[1]->mode );
}

void test_remove_PID_from_stream_and_add(void)
{
    PID_SUPPORTED_STATUS status;
    PID_DATA stream[NUM_CAN_SNIFF];
    status = 0xFF;

    /* Parameter 0 = Engine Oil Temperature */
    stream[0].pid      = MODE1_ENGINE_OIL_TEMPERATURE;
    stream[0].mode     = MODE1;
    stream[0].pid_unit = PID_UNITS_CELCIUS;

    /* Parameter 1 = Turbo inlet pressure */
    stream[1].pid      = MODE1_TURBOCHARGER_COMPRESSOR_INLET_PRESSURE;
    stream[1].mode     = MODE1;
    stream[1].pid_unit = PID_UNITS_KPA;

    /* Parameter 2 = Gauge Brightness */
    stream[2].pid      = SNIFF_GAUGE_BRIGHTNESS;
    stream[2].mode     = SNIFF;
    stream[2].pid_unit = PID_UNITS_PERCENT;

    /* Add Engine Oil Temperature */
    status = 0xFF;
    status = CAN_Sniffer_Add_PID( &sniffer, &stream[0] );

    TEST_ASSERT_EQUAL( PID_SUPPORTED , status );
    TEST_ASSERT_EQUAL( MODE1_ENGINE_OIL_TEMPERATURE , sniffer.stream[0]->pid );
    TEST_ASSERT_EQUAL( MODE1 , sniffer.stream[0]->mode );

    /* Add Turbo inlet pressure */
    status = 0xFF;
    status = CAN_Sniffer_Add_PID( &sniffer, &stream[1] );

    TEST_ASSERT_EQUAL( PID_SUPPORTED , status );
    TEST_ASSERT_EQUAL( MODE1_TURBOCHARGER_COMPRESSOR_INLET_PRESSURE , sniffer.stream[1]->pid );
    TEST_ASSERT_EQUAL( MODE1 , sniffer.stream[1]->mode );

    /* Add Gauge Brightness */
    status = 0xFF;
    status = CAN_Sniffer_Add_PID( &sniffer, &stream[2] );

    TEST_ASSERT_EQUAL( PID_SUPPORTED , status );
    TEST_ASSERT_EQUAL( SNIFF_GAUGE_BRIGHTNESS , sniffer.stream[2]->pid );
    TEST_ASSERT_EQUAL( SNIFF , sniffer.stream[2]->mode );

    /* Remove Turbo inlet pressure */
    status = 0xFF;
    status = CAN_Sniffer_Remove_PID( &sniffer, &stream[1] );

    TEST_ASSERT_EQUAL( PID_SUPPORTED , status );
    TEST_ASSERT_EQUAL( MODE1_ENGINE_OIL_TEMPERATURE , sniffer.stream[0]->pid );
    TEST_ASSERT_EQUAL( MODE1 , sniffer.stream[0]->mode );

    TEST_ASSERT_EQUAL( SNIFF_GAUGE_BRIGHTNESS , sniffer.stream[1]->pid );
    TEST_ASSERT_EQUAL( SNIFF , sniffer.stream[1]->mode );

    /* Add Turbo inlet pressure */
    status = 0xFF;
    status = CAN_Sniffer_Add_PID( &sniffer, &stream[1] );

    TEST_ASSERT_EQUAL( PID_SUPPORTED , status );
    TEST_ASSERT_EQUAL( MODE1_TURBOCHARGER_COMPRESSOR_INLET_PRESSURE , sniffer.stream[2]->pid );
    TEST_ASSERT_EQUAL( MODE1 , sniffer.stream[2]->mode );
}
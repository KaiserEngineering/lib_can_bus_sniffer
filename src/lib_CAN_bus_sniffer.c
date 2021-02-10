/* Author : Matthew Kaiser */
/* @Description: Library to sniff proprietary data from the CAN bus of a vehicle and  *
 * relate it to standard OBD-II parameters (PIDs). The library will require access to *
 * the hardware peripheral's filter configuration and the main loop shall pass the    *
 * relevant packets to the library when received. The library will update the PID     *
 * pointer value as it receives new data.                                             */
#include "lib_CAN_bus_sniffer.h"

/* Number of filters that are supported for the library, this can be different than *
 * the number of filters supported by the hardware.                                 */
#define MAX_NUM_FILTERS 25
uint16_t active_filters[MAX_NUM_FILTERS];
#define RESERVED_FILTER 0xFFFF

uint32_t sniffer_tick = 0;

/* Initialize variables to a known state and verify the proper callbacks have been  *
 * assigned.                                                                        */
void CAN_Sniffer_Initialize( PCAN_SNIFFER_PACKET_MANAGER dev )
{
	/* Clear the PID count */
    dev->num_pids = 0;

    /* Clear the active filters */
    for( uint8_t i = 0; i < MAX_NUM_FILTERS; i++ )
    	active_filters[i] = RESERVED_FILTER;

    /* Set the stream pointer to NULL */
    for( uint8_t i = 0; i < NUM_CAN_SNIFF; i++ )
    	dev->stream[i] = NULL;

    /* Verify the CAN bus filter callback has been assigned */
    if( dev->filter != NULL )
		dev->status |= CAN_SNIFF_INIT;
}

PTR_PID_DATA lib_can_bus_sniffer_get_stream_by_index( PCAN_SNIFFER_PACKET_MANAGER dev, uint8_t index )
{
    return dev->stream[index];
}

/* Ties the CAN bus hardware peripheral to the library and will       *
 * optimize CAN bus filter usage and ensure only one filter is used   *
 * per arbitration ID.                                                */
static void add_filter( PCAN_SNIFFER_PACKET_MANAGER dev, uint16_t id )
{
    if( (dev->status & CAN_SNIFF_INIT) == 0 )
        return;

    /* Check what filters are currently active */
    for( uint8_t i = 0; i < MAX_NUM_FILTERS; i++ )
    {
        /* If the filter ID is already present, then there is no need  *
         * to add another filter. So we can break out of this function */
        if( active_filters[i] == id ) { return; }

        /* Increment until an open slot is available and add the new   *
         * filter to the device.                                       */
        else if( active_filters[i] == RESERVED_FILTER )
        {
        	/* Copy the ID to the filter slot */
            active_filters[i] = id;

            /* Request the filter from the hardware peripheral.       *
             * TODO: Error handle, what if the hardware fails or if   *
             * the hardware runs out of mailboxes?                    */
            dev->filter( id );

            return;
        }
    }
}

/* Verify that that the PID is supported. */
PID_SUPPORTED_STATUS CAN_Sniffer_PID_Supported( PTR_PID_DATA pid )
{
    switch( pid->pid )
    {
        #ifdef FORD_FOCUS_STRS_2013_2018

        #ifdef SNIFF_ENGINE_RPM_PID
        case SNIFF_ENGINE_RPM_PID:
        #endif

        #ifdef SNIFF_ACCEL_PEDAL_POS_PID
        case SNIFF_ACCEL_PEDAL_POS_PID:
        #endif

        #ifdef SNIFF_ENGINE_OIL_TEMP_PID
        case SNIFF_ENGINE_OIL_TEMP_PID:
        #endif

        #ifdef SNIFF_BOOST_PRESSURE_PID
        case SNIFF_BOOST_PRESSURE_PID:
        #endif

        #ifdef SNIFF_GAUGE_BRIGHTNESS_PID
        case SNIFF_GAUGE_BRIGHTNESS_PID:
        #endif
            return PID_SUPPORTED;

        #endif

        default:
            return PID_NOT_SUPPORTED;
    }
}

/* Add a PID to the packet manager to be streamed. This will return   *
 * @PID_SUPPORTED_STATUS to verify if the PID was or was not added.   *
 * Upon adding a supported PID, the library will request a hardware   *
 * filter if necessary (see add_filter)                               */
PID_SUPPORTED_STATUS CAN_Sniffer_Add_PID( PCAN_SNIFFER_PACKET_MANAGER dev, PTR_PID_DATA pid )
{
	/* Check to see if the PID can be sniffed by the library          */
	if( CAN_Sniffer_PID_Supported( pid ) == PID_SUPPORTED )
	{
		/* Determine what arbitration ID needs to be monitored to     *
		 * obtain the PID data.	                                      */
		switch( pid->pid )
		{
            #ifdef FORD_FOCUS_STRS_2013_2018

			#ifdef SNIFF_ENGINE_RPM_PID
			case SNIFF_ENGINE_RPM_PID:
				add_filter( dev, SNIFF_ENGINE_RPM_ID );
				pid->base_unit = PID_UNITS_RPM;
				break;
			#endif

			#ifdef SNIFF_ACCEL_PEDAL_POS_PID
			case SNIFF_ACCEL_PEDAL_POS_PID:
				add_filter( dev, SNIFF_ACCEL_PEDAL_POS_ID );
				pid->base_unit = PID_UNITS_PERCENT;
				break;
			#endif

			#ifdef SNIFF_ENGINE_OIL_TEMP_PID
			case SNIFF_ENGINE_OIL_TEMP_PID:
				add_filter( dev, SNIFF_ENGINE_OIL_TEMP_ID );
				pid->base_unit = PID_UNITS_CELCIUS;
				break;
			#endif

			#ifdef SNIFF_BOOST_PRESSURE_PID
			case SNIFF_BOOST_PRESSURE_PID:
				add_filter( dev, SNIFF_BOOST_PRESSURE_ID );
				pid->base_unit = PID_UNITS_KPA;
				break;
			#endif

            #ifdef SNIFF_GAUGE_BRIGHTNESS_PID
            case SNIFF_GAUGE_BRIGHTNESS_PID:
                add_filter( dev, SNIFF_GAUGE_BRIGHTNESS_ID );
                pid->base_unit = PID_UNITS_PERCENT;
                break;
            #endif

            #endif
		}

		dev->stream[dev->num_pids] = pid;

		/* Increment the number of PIDs being streamed */
		dev->num_pids++;

		/* PID is supported and the filter is configured */
		return PID_SUPPORTED;
	}

	/* This PID is not supported, no filters were added */
	else { return PID_NOT_SUPPORTED; }
}

PID_SUPPORTED_STATUS CAN_Sniffer_Remove_PID( PCAN_SNIFFER_PACKET_MANAGER dev, PTR_PID_DATA pid )
{
    /* Cycle through all the PIDs to find which one must be removed */
    for( uint8_t i = 0; i < dev->num_pids; i++ )
    {
        /* If found, pop that pointer reference */
        if( dev->stream[i] == pid )
        {
            if( dev->num_pids > 1 )
            {
                for( uint8_t j = i + 1; j < dev->num_pids; j++ )
                {
                    dev->stream[j - 1] = dev->stream[j];
                    dev->stream[j] = NULL;
                }
            }

            /* Remove the PID */
            dev->num_pids--;
        }
    }

    return PID_SUPPORTED;
}

void CAN_Sniffer_Add_Packet( PCAN_SNIFFER_PACKET_MANAGER dev, uint16_t arbitration_id, uint8_t* packet_data )
{
	/* Check all of the PIDs */
    for( uint8_t i = 0; i < dev->num_pids; i++ )
    {
    	if( dev->stream[i] != NULL )
    	{
            uint8_t timestamp_flag = 1;

			switch( arbitration_id )
			{
                #ifdef FORD_FOCUS_STRS_2013_2018

                #ifdef SNIFF_ENGINE_RPM_PID
				case SNIFF_ENGINE_RPM_ID:
					/* Engine RPM */
					if( (dev->stream[i]->pid == MODE1_ENGINE_RPM) && (dev->stream[i]->mode == MODE1) )
						dev->stream[i]->pid_value = (float)(((uint32_t)(packet_data[4] & 0xF) << 8) | (uint32_t)(packet_data[5])) * (float)2;
					break;
                #endif

                #ifdef SNIFF_ACCEL_PEDAL_POS_PID
				case SNIFF_ACCEL_PEDAL_POS_ID:
					/* Accelerator Pedal */
					if( (dev->stream[i]->pid == MODE1_REL_ACCELERATOR_PEDAL_POS) && (dev->stream[i]->mode == MODE1) )
						dev->stream[i]->pid_value = (float)(((uint32_t)(packet_data[0] & 0x3) << 8) | (uint32_t)(packet_data[1])) / (float)10;
					break;
                #endif

                #ifdef SNIFF_ENGINE_OIL_TEMP_PID
				case SNIFF_ENGINE_OIL_TEMP_ID:
					/* Engine Oil Temperature */
					if( (dev->stream[i]->pid == MODE1_ENGINE_OIL_TEMPERATURE) && (dev->stream[i]->mode == MODE1) ) {
					    dev->stream[i]->pid_value = (float)packet_data[7] - (float)60;
					}

					/* Boost Pressure */
					else if( (dev->stream[i]->pid == MODE1_TURBO_INLET_PRESSURE) && (dev->stream[i]->mode == MODE1) ) {
						dev->stream[i]->pid_value = (float)packet_data[5];
					}
					break;
                #endif

                #ifdef SNIFF_GAUGE_BRIGHTNESS_PID
				case SNIFF_GAUGE_BRIGHTNESS_ID:
				    if( (dev->stream[i]->pid == SNIFF_GAUGE_BRIGHTNESS) && (dev->stream[i]->mode == SNIFF) )
				        dev->stream[i]->pid_value = (float)(packet_data[0] & 0x1F);
				    break;
                #endif

                #endif

                default:
                    timestamp_flag = 0;
                    break;
			}

            if( timestamp_flag )
                dev->stream[i]->timestamp = sniffer_tick;
    	}
    }
}

void CAN_Sniffer_tick( void )
{
    sniffer_tick++;
}

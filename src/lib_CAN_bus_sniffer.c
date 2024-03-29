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
    switch( pid->mode )
    {
        case MODE1:
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
            break;

            case SNIFF:
                switch( pid->pid )
                {
                    #ifdef FORD_FOCUS_STRS_2013_2018

                    #ifdef SNIFF_GAUGE_BRIGHTNESS_PID
                    case SNIFF_GAUGE_BRIGHTNESS_PID:
                    #endif

                    #ifdef SNIFF_BRAKE_PEDAL_STATUS_PID
                    case SNIFF_BRAKE_PEDAL_STATUS_PID:
                    #endif

                    #ifdef SNIFF_EMERGENCY_BRAKE_STATUS_PID
                    case SNIFF_EMERGENCY_BRAKE_STATUS_PID:
                    #endif

                    #ifdef SNIFF_REVERSE_STATUS_PID
                    case SNIFF_REVERSE_STATUS_PID:
                    #endif

                    #ifdef SNIFF_CRUISE_CONTROL_ON_BUTTON_PID
                    case SNIFF_CRUISE_CONTROL_ON_BUTTON_PID:
                    #endif

                    #ifdef SNIFF_CRUISE_CONTROL_OFF_BUTTON_PID
                    case SNIFF_CRUISE_CONTROL_OFF_BUTTON_PID:
                    #endif

                    #ifdef SNIFF_CRUISE_CONTROL_SET_PLUS_BUTTON_PID
                    case SNIFF_CRUISE_CONTROL_SET_PLUS_BUTTON_PID:
                    #endif

                    #ifdef SNIFF_CRUISE_CONTROL_SET_MINUS_BUTTON_PID
                    case SNIFF_CRUISE_CONTROL_SET_MINUS_BUTTON_PID:
                    #endif

                    #ifdef SNIFF_CRUISE_CONTROL_RES_BUTTON_PID
                    case SNIFF_CRUISE_CONTROL_RES_BUTTON_PID:
                    #endif

                    #ifdef SNIFF_CRUISE_CONTROL_CAN_BUTTON_PID
                    case SNIFF_CRUISE_CONTROL_CAN_BUTTON_PID:
                    #endif

                    #ifdef SNIFF_LATERAL_ACCELERATION_PID
                    case SNIFF_LATERAL_ACCELERATION_PID:
                    #endif

                    #ifdef SNIFF_LONGITUDINAL_ACCELERATION_PID
                    case SNIFF_LONGITUDINAL_ACCELERATION_PID:
                    #endif

                        return PID_SUPPORTED;

                    #endif

                    default:
                        return PID_NOT_SUPPORTED;
                }
                break;

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

			#if defined(SNIFF_ENGINE_RPM_PID) || !defined(LIMIT_PIDS)
			case SNIFF_ENGINE_RPM_PID:
				add_filter( dev, SNIFF_ENGINE_RPM_ID );
				pid->base_unit = PID_UNITS_RPM;
				break;
			#endif

			#if defined(SNIFF_ACCEL_PEDAL_POS_PID) || !defined(LIMIT_PIDS)
			case SNIFF_ACCEL_PEDAL_POS_PID:
				add_filter( dev, SNIFF_ACCEL_PEDAL_POS_ID );
				pid->base_unit = PID_UNITS_PERCENT;
				break;
			#endif

			#if defined(SNIFF_ENGINE_OIL_TEMP_PID) || !defined(LIMIT_PIDS)
			case SNIFF_ENGINE_OIL_TEMP_PID:
				add_filter( dev, SNIFF_ENGINE_OIL_TEMP_ID );
				pid->base_unit = PID_UNITS_CELSIUS;
				break;
			#endif

			#if defined(SNIFF_BOOST_PRESSURE_PID) || !defined(LIMIT_PIDS)
			case SNIFF_BOOST_PRESSURE_PID:
				add_filter( dev, SNIFF_BOOST_PRESSURE_ID );
				pid->base_unit = PID_UNITS_KPA;
				break;
			#endif

            #if defined(SNIFF_GAUGE_BRIGHTNESS_PID) || !defined(LIMIT_PIDS)
            case SNIFF_GAUGE_BRIGHTNESS_PID:
                add_filter( dev, SNIFF_GAUGE_BRIGHTNESS_ID );
                pid->base_unit = PID_UNITS_PERCENT;
                break;
            #endif

            #if defined(SNIFF_BRAKE_PEDAL_STATUS_PID) || !defined(LIMIT_PIDS)
            case SNIFF_BRAKE_PEDAL_STATUS_PID:
                add_filter( dev, SNIFF_BRAKE_PEDAL_STATUS_ID );
                pid->base_unit = PID_UNITS_NONE;
                break;
            #endif

            #if defined(SNIFF_EMERGENCY_BRAKE_STATUS_PID) || !defined(LIMIT_PIDS)
            case SNIFF_EMERGENCY_BRAKE_STATUS_PID:
                add_filter( dev, SNIFF_EMERGENCY_BRAKE_STATUS_ID );
                pid->base_unit = PID_UNITS_NONE;
                break;
            #endif

            #if defined(SNIFF_REVERSE_STATUS_PID) || !defined(LIMIT_PIDS)
            case SNIFF_REVERSE_STATUS_PID:
                add_filter( dev, SNIFF_REVERSE_STATUS_ID );
                pid->base_unit = PID_UNITS_NONE;
                break;
            #endif

            #if defined(SNIFF_CRUISE_CONTROL_ON_BUTTON_PID) || !defined(LIMIT_PIDS)
            case SNIFF_CRUISE_CONTROL_ON_BUTTON_PID:
                add_filter( dev, SNIFF_CRUISE_CONTROL_ON_BUTTON_ID );
                pid->base_unit = PID_UNITS_NONE;
                break;
            #endif

            #if defined(SNIFF_CRUISE_CONTROL_OFF_BUTTON_PID) || !defined(LIMIT_PIDS)
            case SNIFF_CRUISE_CONTROL_OFF_BUTTON_PID:
                add_filter( dev, SNIFF_CRUISE_CONTROL_OFF_BUTTON_ID );
                pid->base_unit = PID_UNITS_NONE;
                break;
            #endif

            #if defined(SNIFF_CRUISE_CONTROL_SET_PLUS_BUTTON_PID) || !defined(LIMIT_PIDS)
            case SNIFF_CRUISE_CONTROL_SET_PLUS_BUTTON_PID:
                add_filter( dev, SNIFF_CRUISE_CONTROL_SET_PLUS_BUTTON_ID );
                pid->base_unit = PID_UNITS_NONE;
                break;
            #endif

            #if defined(SNIFF_CRUISE_CONTROL_SET_MINUS_BUTTON_PID) || !defined(LIMIT_PIDS)
            case SNIFF_CRUISE_CONTROL_SET_MINUS_BUTTON_PID:
                add_filter( dev, SNIFF_CRUISE_CONTROL_SET_MINUS_BUTTON_ID );
                pid->base_unit = PID_UNITS_NONE;
                break;
            #endif

            #if defined(SNIFF_CRUISE_CONTROL_RES_BUTTON_PID) || !defined(LIMIT_PIDS)
            case SNIFF_CRUISE_CONTROL_RES_BUTTON_PID:
                add_filter( dev, SNIFF_CRUISE_CONTROL_RES_BUTTON_ID );
                pid->base_unit = PID_UNITS_NONE;
                break;
            #endif

            #if defined(SNIFF_CRUISE_CONTROL_CAN_BUTTON_PID) || !defined(LIMIT_PIDS)
            case SNIFF_CRUISE_CONTROL_CAN_BUTTON_PID:
                add_filter( dev, SNIFF_CRUISE_CONTROL_CAN_BUTTON_ID );
                pid->base_unit = PID_UNITS_NONE;
                break;
            #endif

            #if defined(SNIFF_LONGITUDINAL_ACCELERATION_SUPPORTED) || !defined(LIMIT_PIDS)
            case SNIFF_LATERAL_ACCELERATION_PID:
                add_filter( dev, SNIFF_LATERAL_ACCELERATION_ID );
                pid->base_unit = PID_UNITS_G_FORCE;
                break;
            #endif

            #if defined(SNIFF_LONGITUDINAL_ACCELERATION_SUPPORTED) || !defined(LIMIT_PIDS)
            case SNIFF_LONGITUDINAL_ACCELERATION_PID:
                add_filter( dev, SNIFF_LONGITUDINAL_ACCELERATION_ID );
                pid->base_unit = PID_UNITS_G_FORCE;
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
    for( uint8_t index = 0; index < dev->num_pids; index++ )
    {
        /* If found, pop that pointer reference */
        if( dev->stream[index] == pid )
        {
            if( dev->num_pids > 1 )
            {
                for( uint8_t i = index; i < dev->num_pids; i++ ) {
                    dev->stream[i] = dev->stream[i + 1];
                    dev->stream[i+1] = NULL;
                }
            }

            /* Remove the PID */
            dev->num_pids--;
        }
    }

    return PID_SUPPORTED;
}

void CAN_Sniffer_Add_Packet( PCAN_SNIFFER_PACKET_MANAGER dev, uint16_t arbitration_id, uint8_t* data )
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
				case 0x090:
					/* Engine RPM */
					if( (dev->stream[i]->pid == MODE1_ENGINE_SPEED) && (dev->stream[i]->mode == MODE1) )
						dev->stream[i]->pid_value = (float)(((uint32_t)(data[4] & 0xF) << 8) | (uint32_t)(data[5])) * (float)2;
					break;
                #endif

                #if defined(SNIFF_ACCEL_PEDAL_POS_PID)    || \
                    defined(SNIFF_BRAKE_PEDAL_STATUS_PID) || \
                    defined(SNIFF_REVERSE_STATUS_PID)
				case 0x080:

					/* Accelerator Pedal */
					if( (dev->stream[i]->pid == MODE1_RELATIVE_ACCELERATOR_PEDAL_POSITION) && (dev->stream[i]->mode == MODE1) )
						dev->stream[i]->pid_value = (float)(((uint32_t)(data[0] & 0x3) << 8) | (uint32_t)(data[1])) / (float)10;

					/* Brake Pedal Status */
					else if( (dev->stream[i]->pid == SNIFF_BRAKE_PEDAL_STATUS) && (dev->stream[i]->mode == SNIFF) )
                        dev->stream[i]->pid_value = (float)((data[0] & 0x04) > 0);

                    /* Reverse Status */
                    else if( (dev->stream[i]->pid == SNIFF_REVERSE_STATUS_PID) && (dev->stream[i]->mode == SNIFF) )
                        dev->stream[i]->pid_value = (float)((data[0] & 0x20) > 0);

					break;
                #endif

                #if defined(SNIFF_ENGINE_OIL_TEMP_PID) || \
                    defined(SNIFF_BOOST_PRESSURE_PID)
				case 0x0F8:
					/* Engine Oil Temperature */
					if( (dev->stream[i]->pid == MODE1_ENGINE_OIL_TEMPERATURE) && (dev->stream[i]->mode == MODE1) ) {
					    dev->stream[i]->pid_value = (float)data[7] - (float)60;
					}

					/* Boost Pressure */
					else if( (dev->stream[i]->pid == MODE1_TURBOCHARGER_COMPRESSOR_INLET_PRESSURE) && (dev->stream[i]->mode == MODE1) ) {
						dev->stream[i]->pid_value = (float)data[5];
					}

					break;
                #endif

                #if defined(SNIFF_GAUGE_BRIGHTNESS_PID) || \
					defined(SNIFF_EMERGENCY_BRAKE_STATUS_PID)
				case 0x0C8:

				    /* Gauge Brightness */
				    if( (dev->stream[i]->pid == SNIFF_GAUGE_BRIGHTNESS) && (dev->stream[i]->mode == SNIFF) ) {
				        dev->stream[i]->pid_value = (float)(data[0] & 0x1F);
				    }

                    /* E-brake Status */
                    else if( (dev->stream[i]->pid == SNIFF_EMERGENCY_BRAKE_STATUS) && (dev->stream[i]->mode == SNIFF) ) {
                        dev->stream[i]->pid_value = (float)((data[3] & 0x40) > 0);
                    }

				    break;
                #endif

                #if defined(SNIFF_CRUISE_CONTROL_ON_BUTTON_PID)        || \
                    defined(SNIFF_CRUISE_CONTROL_OFF_BUTTON_PID)       || \
                    defined(SNIFF_CRUISE_CONTROL_SET_PLUS_BUTTON_PID)  || \
                    defined(SNIFF_CRUISE_CONTROL_SET_MINUS_BUTTON_PID) || \
                    defined(SNIFF_CRUISE_CONTROL_RES_BUTTON_PID)       || \
                    defined(SNIFF_CRUISE_CONTROL_CAN_BUTTON_PID)
                case 0x030:

                    /* Cruise Control OFF button Status */
                    if( (dev->stream[i]->pid == SNIFF_CRUISE_CONTROL_ON_BUTTON) && (dev->stream[i]->mode == SNIFF) ) {
                        dev->stream[i]->pid_value = (float)((data[5] & 0x01) > 0);
                    }

                    /* Cruise Control ON Button Status */
                    else if( (dev->stream[i]->pid == SNIFF_CRUISE_CONTROL_OFF_BUTTON) && (dev->stream[i]->mode == SNIFF) ) {
                        dev->stream[i]->pid_value = (float)((data[5] & 0x02) > 0);
                    }

                    /* Cruise Control SET+ Button Status */
                    else if( (dev->stream[i]->pid == SNIFF_CRUISE_CONTROL_SET_PLUS_BUTTON) && (dev->stream[i]->mode == SNIFF) ) {
                        dev->stream[i]->pid_value = (float)((data[5] & 0x80) > 0);
                    }

                    /* Cruise Control SET- Button Status */
                    else if( (dev->stream[i]->pid == SNIFF_CRUISE_CONTROL_SET_MINUS_BUTTON) && (dev->stream[i]->mode == SNIFF) ) {
                        dev->stream[i]->pid_value = (float)((data[4] & 0x01) > 0);
                    }

                    /* Cruise Control RES Button Status */
                    else if( (dev->stream[i]->pid == SNIFF_CRUISE_CONTROL_RES_BUTTON) && (dev->stream[i]->mode == SNIFF) ) {
                        dev->stream[i]->pid_value = (float)((data[5] & 0x20) > 0);
                    }

                    /* Cruise Control CAN Button Status */
                    else if( (dev->stream[i]->pid == SNIFF_CRUISE_CONTROL_CAN_BUTTON) && (dev->stream[i]->mode == SNIFF) ) {
                        dev->stream[i]->pid_value = (float)((data[5] & 0x10) > 0);
                    }

                    break;
                #endif

                #ifdef SNIFF_LONGITUDINAL_ACCELERATION_PID
                case 0x160:
                    /* Longitudinal Acceleration */
                    if( (dev->stream[i]->pid == SNIFF_LONGITUDINAL_ACCELERATION) && (dev->stream[i]->mode == SNIFF) )
                        if( ((data[6] & 0x3) != 0x3) & (data[7] != 0xFF) )
                            dev->stream[i]->pid_value = (float)(((((uint32_t)(data[6] & 0x3) << 8) | (uint32_t)(data[7])) * (float)0.00390625) - 2);
                    break;
                #endif

                #ifdef SNIFF_LATERAL_ACCELERATION_PID
                case 0x180:
                    /* Lateral Acceleration */
                    if( (dev->stream[i]->pid == SNIFF_LATERAL_ACCELERATION) && (dev->stream[i]->mode == SNIFF) )
                        if( ((data[2] & 0x3) != 0x3) & (data[3] != 0xFF) )
                            dev->stream[i]->pid_value = (float)(((((uint32_t)(data[2] & 0x3) << 8) | (uint32_t)(data[3])) * (float)0.00390625) - 2);
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

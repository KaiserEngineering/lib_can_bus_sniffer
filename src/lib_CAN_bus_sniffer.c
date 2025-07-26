/* Author : Matthew Kaiser */
/* @Description: Library to sniff proprietary data from the CAN bus of a vehicle and  *
 * relate it to standard OBD-II parameters (PIDs). The library will require access to *
 * the hardware peripheral's filter configuration and the main loop shall pass the    *
 * relevant packets to the library when received. The library will update the PID     *
 * pointer value as it receives new data.                                             */
#include "lib_CAN_bus_sniffer.h"

/* Number of filters that are supported for the library, this can be different than *
 * the number of filters supported by the hardware.                                 */
#ifndef MAX_CAN_FILTERS
#define MAX_CAN_FILTERS 25
#endif
uint16_t active_filters[MAX_CAN_FILTERS];
#define RESERVED_FILTER 0xFFFF

uint32_t sniffer_tick = 0;

/* Initialize variables to a known state and verify the proper callbacks have been  *
 * assigned.                                                                        */
void CAN_Sniffer_Initialize( PCAN_SNIFFER_PACKET_MANAGER dev )
{
	/* Clear the PID count */
    dev->num_pids = 0;

    /* Clear the active filters */
    for( uint8_t i = 0; i < MAX_CAN_FILTERS; i++ )
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
    for( uint8_t i = 0; i < MAX_CAN_FILTERS; i++ )
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
            if( dev->filter != NULL )
            	dev->filter( id, 1 );

            return;
        }
    }
}

/* Removes a CAN filter from the hardware and frees its slot in the manager */
static void remove_filter( PCAN_SNIFFER_PACKET_MANAGER dev, uint16_t id )
{
    if( (dev->status & CAN_SNIFF_INIT) == 0 )
        return;

    // Search for the filter ID in active filters
    for( uint8_t i = 0; i < MAX_CAN_FILTERS; i++ )
    {
        if( active_filters[i] == id )
        {
            // Request filter removal from hardware
            if( dev->filter != NULL )
                dev->filter( id, 0 );

            // Mark the slot as available
            active_filters[i] = RESERVED_FILTER;

            return;
        }
    }
}

static void config_filter( PCAN_SNIFFER_PACKET_MANAGER dev, uint16_t id, uint8_t enable )
{
	if(enable)
		add_filter(dev, id);
	else
		remove_filter(dev, id);
}

/* Verify that that the PID is supported. */
PID_SUPPORTED_STATUS CAN_Sniffer_PID_Supported( PTR_PID_DATA pid )
{
    switch( get_mode_by_uuid(pid->pid_uuid) )
    {
        case MODE1:
            switch( get_pid_by_uuid(pid->pid_uuid) )
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
                switch( get_pid_by_uuid(pid->pid_uuid) )
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

static PID_SUPPORTED_STATUS CAN_Sniffer_Edit_PID( PCAN_SNIFFER_PACKET_MANAGER dev, PTR_PID_DATA pid, uint8_t enable )
{
	/* Check to see if the PID can be sniffed by the library          */
	if( CAN_Sniffer_PID_Supported( pid ) == PID_SUPPORTED )
	{
		/* Determine what arbitration ID needs to be monitored to     *
		 * obtain the PID data.	                                      */
		switch( get_pid_by_uuid(pid->pid_uuid) )
		{
            #ifdef FORD_FOCUS_STRS_2013_2018

			#if defined(SNIFF_ENGINE_RPM_PID) || !defined(LIMIT_PIDS)
			case SNIFF_ENGINE_RPM_PID:
				config_filter( dev, SNIFF_ENGINE_RPM_ID, enable );
				pid->base_unit = PID_UNITS_RPM;
				break;
			#endif

			#if defined(SNIFF_ACCEL_PEDAL_POS_PID) || !defined(LIMIT_PIDS)
			case SNIFF_ACCEL_PEDAL_POS_PID:
				config_filter( dev, SNIFF_ACCEL_PEDAL_POS_ID, enable );
				pid->base_unit = PID_UNITS_PERCENT;
				break;
			#endif

			#if defined(SNIFF_ENGINE_OIL_TEMP_PID) || !defined(LIMIT_PIDS)
			case SNIFF_ENGINE_OIL_TEMP_PID:
				config_filter( dev, SNIFF_ENGINE_OIL_TEMP_ID, enable );
				pid->base_unit = PID_UNITS_CELSIUS;
				break;
			#endif

			#if defined(SNIFF_BOOST_PRESSURE_PID) || !defined(LIMIT_PIDS)
			case SNIFF_BOOST_PRESSURE_PID:
				config_filter( dev, SNIFF_BOOST_PRESSURE_ID, enable );
				pid->base_unit = PID_UNITS_KPA;
				break;
			#endif

            #if defined(SNIFF_GAUGE_BRIGHTNESS_PID) || !defined(LIMIT_PIDS)
            case SNIFF_GAUGE_ILLUM_LEVEL_PID:
                config_filter( dev, SNIFF_GAUGE_BRIGHTNESS_ID, enable );
                pid->base_unit = PID_UNITS_PERCENT;
                break;
            #endif

            #if defined(SNIFF_BRAKE_PEDAL_STATUS_PID) || !defined(LIMIT_PIDS)
            case SNIFF_BRAKE_PEDAL_STATUS_PID:
                config_filter( dev, SNIFF_BRAKE_PEDAL_STATUS_ID, enable );
                pid->base_unit = PID_UNITS_NONE;
                break;
            #endif

            #if defined(SNIFF_EMERGENCY_BRAKE_STATUS_PID) || !defined(LIMIT_PIDS)
            case SNIFF_EMERGENCY_BRAKE_STATUS_PID:
                config_filter( dev, SNIFF_EMERGENCY_BRAKE_STATUS_ID, enable );
                pid->base_unit = PID_UNITS_NONE;
                break;
            #endif

            #if defined(SNIFF_REVERSE_STATUS_PID) || !defined(LIMIT_PIDS)
            case SNIFF_REVERSE_STATUS_PID:
                config_filter( dev, SNIFF_REVERSE_STATUS_ID, enable );
                pid->base_unit = PID_UNITS_NONE;
                break;
            #endif

            #if defined(SNIFF_CRUISE_CONTROL_ON_BUTTON_PID) || !defined(LIMIT_PIDS)
            case SNIFF_CRUISE_CONTROL_ON_BUTTON_PID:
                config_filter( dev, SNIFF_CRUISE_CONTROL_ON_BUTTON_ID, enable );
                pid->base_unit = PID_UNITS_NONE;
                break;
            #endif

            #if defined(SNIFF_CRUISE_CONTROL_OFF_BUTTON_PID) || !defined(LIMIT_PIDS)
            case SNIFF_CRUISE_CONTROL_OFF_BUTTON_PID:
                config_filter( dev, SNIFF_CRUISE_CONTROL_OFF_BUTTON_ID, enable );
                pid->base_unit = PID_UNITS_NONE;
                break;
            #endif

            #if defined(SNIFF_CRUISE_CONTROL_SET_PLUS_BUTTON_PID) || !defined(LIMIT_PIDS)
            case SNIFF_CRUISE_CONTROL_SET_PLUS_BUTTON_PID:
                config_filter( dev, SNIFF_CRUISE_CONTROL_SET_PLUS_BUTTON_ID, enable );
                pid->base_unit = PID_UNITS_NONE;
                break;
            #endif

            #if defined(SNIFF_CRUISE_CONTROL_SET_MINUS_BUTTON_PID) || !defined(LIMIT_PIDS)
            case SNIFF_CRUISE_CONTROL_SET_MINUS_BUTTON_PID:
                config_filter( dev, SNIFF_CRUISE_CONTROL_SET_MINUS_BUTTON_ID, enable );
                pid->base_unit = PID_UNITS_NONE;
                break;
            #endif

            #if defined(SNIFF_CRUISE_CONTROL_RES_BUTTON_PID) || !defined(LIMIT_PIDS)
            case SNIFF_CRUISE_CONTROL_RES_BUTTON_PID:
                config_filter( dev, SNIFF_CRUISE_CONTROL_RES_BUTTON_ID, enable );
                pid->base_unit = PID_UNITS_NONE;
                break;
            #endif

            #if defined(SNIFF_CRUISE_CONTROL_CAN_BUTTON_PID) || !defined(LIMIT_PIDS)
            case SNIFF_CRUISE_CONTROL_CAN_BUTTON_PID:
                config_filter( dev, SNIFF_CRUISE_CONTROL_CAN_BUTTON_ID, enable );
                pid->base_unit = PID_UNITS_NONE;
                break;
            #endif

            #if defined(SNIFF_LONGITUDINAL_ACCELERATION_SUPPORTED) || !defined(LIMIT_PIDS)
            case SNIFF_LATERAL_ACCELERATION_PID:
                config_filter( dev, SNIFF_LATERAL_ACCELERATION_ID, enable );
                pid->base_unit = PID_UNITS_G_FORCE;
                break;
            #endif

            #if defined(SNIFF_LONGITUDINAL_ACCELERATION_SUPPORTED) || !defined(LIMIT_PIDS)
            case SNIFF_LONGITUDINAL_ACCELERATION_PID:
                config_filter( dev, SNIFF_LONGITUDINAL_ACCELERATION_ID, enable );
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

/* Add a PID to the packet manager to be streamed. This will return   *
 * @PID_SUPPORTED_STATUS to verify if the PID was or was not added.   *
 * Upon adding a supported PID, the library will request a hardware   *
 * filter if necessary (see add_filter)                               */
PID_SUPPORTED_STATUS CAN_Sniffer_Add_PID( PCAN_SNIFFER_PACKET_MANAGER dev, PTR_PID_DATA pid )
{
	return CAN_Sniffer_Edit_PID(dev, pid, 1);
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

            /* Remove the filter */
            CAN_Sniffer_Edit_PID( dev, pid, 0 );

            /* Remove the PID */
            dev->num_pids--;
        }
    }

    return PID_SUPPORTED;
}

static void process_change( PTR_PID_DATA pid )
{
	pid->timestamp = sniffer_tick;
	convert_units( pid->base_unit, pid->pid_unit, &pid->pid_value);
}

void CAN_Sniffer_Add_Packet( PCAN_SNIFFER_PACKET_MANAGER dev, uint16_t arbitration_id, uint8_t* data )
{
	/* Check all of the PIDs */
    for( uint8_t i = 0; i < dev->num_pids; i++ )
    {
    	if( dev->stream[i] != NULL )
    	{
			switch( arbitration_id )
			{
                #ifdef FORD_FOCUS_STRS_2013_2018

                #ifdef SNIFF_ENGINE_RPM_PID
				case 0x090:
					/* Engine RPM */
					if( dev->stream[i]->pid_uuid == MODE1_ENGINE_SPEED_UUID ) {
						dev->stream[i]->pid_value = (float)(((uint32_t)(data[4] & 0xF) << 8) | (uint32_t)(data[5])) * (float)2;
						process_change(dev->stream[i]);
					}
					break;
                #endif

                #if defined(SNIFF_ACCEL_PEDAL_POS_PID)    || \
                    defined(SNIFF_BRAKE_PEDAL_STATUS_PID) || \
                    defined(SNIFF_REVERSE_STATUS_PID)
				case 0x080:

					/* Accelerator Pedal */
					if( dev->stream[i]->pid_uuid == MODE1_ACCEL_PEDAL_POS_UUID ) {
						dev->stream[i]->pid_value = (float)(((uint32_t)(data[0] & 0x3) << 8) | (uint32_t)(data[1])) / (float)10;
						process_change(dev->stream[i]);
					}

					/* Brake Pedal Status */
					else if( dev->stream[i]->pid_uuid == SNIFF_BRAKE_PEDAL_STATUS_UUID ) {
                        dev->stream[i]->pid_value = (float)((data[0] & 0x04) > 0);
                        process_change(dev->stream[i]);
					}

                    /* Reverse Status */
                    else if( dev->stream[i]->pid_uuid == SNIFF_REVERSE_STATUS_UUID ) {
                        dev->stream[i]->pid_value = (float)((data[0] & 0x20) > 0);
                        process_change(dev->stream[i]);
                    }

					break;
                #endif

                #if defined(SNIFF_ENGINE_OIL_TEMP_PID) || \
                    defined(SNIFF_BOOST_PRESSURE_PID)
				case 0x0F8:
					/* Engine Oil Temperature */
					if( dev->stream[i]->pid_uuid == MODE1_OIL_TEMP_UUID ) {
					    dev->stream[i]->pid_value = (float)data[7] - (float)60;
					    process_change(dev->stream[i]);
					}

					/* Boost Pressure */
					else if( dev->stream[i]->pid_uuid == MODE1_BOOST_PID ) {
						dev->stream[i]->pid_value = (float)data[5];
						process_change(dev->stream[i]);
					}

					break;
                #endif

                #if defined(SNIFF_GAUGE_ILLUM_LEVEL_UUID) || \
					defined(SNIFF_EMERGENCY_BRAKE_STATUS_PID)
				case 0x0C8:

				    /* Gauge Brightness */
				    if( dev->stream[i]->pid_uuid == SNIFF_GAUGE_ILLUM_LEVEL_UUID ) {
				        dev->stream[i]->pid_value = (float)(data[0] & 0x1F);
				        process_change(dev->stream[i]);
				    }

                    /* E-brake Status */
                    else if( dev->stream[i]->pid_uuid == SNIFF_EMERGENCY_BRAKE_STATUS_UUID ) {
                        dev->stream[i]->pid_value = (float)((data[3] & 0x40) > 0);
                        process_change(dev->stream[i]);
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

                    /* Cruise Control ON button Status */
                    if( dev->stream[i]->pid_uuid == SNIFF_CRUISE_CONTROL_ON_BUTTON_UUID ) {
                        dev->stream[i]->pid_value = (float)((data[5] & 0x01) > 0);
                        process_change(dev->stream[i]);
                    }

                    /* Cruise Control OFF Button Status */
                    else if( dev->stream[i]->pid_uuid == SNIFF_CRUISE_CONTROL_OFF_BUTTON_UUID ) {
                        dev->stream[i]->pid_value = (float)((data[5] & 0x02) > 0);
                        process_change(dev->stream[i]);
                    }

                    /* Cruise Control SET+ Button Status */
                    else if( dev->stream[i]->pid_uuid == SNIFF_CRUISE_CONTROL_SET_PLUS_BUTTON_UUID ) {
                        dev->stream[i]->pid_value = (float)((data[5] & 0x80) > 0);
                        process_change(dev->stream[i]);
                    }

                    /* Cruise Control SET- Button Status */
                    else if( dev->stream[i]->pid_uuid == SNIFF_CRUISE_CONTROL_SET_MINUS_BUTTON_UUID ) {
                        dev->stream[i]->pid_value = (float)((data[4] & 0x01) > 0);
                        process_change(dev->stream[i]);
                    }

                    /* Cruise Control RES Button Status */
                    else if( dev->stream[i]->pid_uuid == SNIFF_CRUISE_CONTROL_RES_BUTTON_UUID ) {
                        dev->stream[i]->pid_value = (float)((data[5] & 0x20) > 0);
                        process_change(dev->stream[i]);
                    }

                    /* Cruise Control CAN Button Status */
                    else if( dev->stream[i]->pid_uuid == SNIFF_CRUISE_CONTROL_CAN_BUTTON_UUID ) {
                        dev->stream[i]->pid_value = (float)((data[5] & 0x10) > 0);
                        process_change(dev->stream[i]);
                    }

                    break;
                #endif

                #ifdef SNIFF_LONGITUDINAL_ACCELERATION_PID
                case 0x160:
                    /* Longitudinal Acceleration */
                    if( dev->stream[i]->pid_uuid == SNIFF_LONGITUDINAL_ACCELERATION_UUID ) {
                        if( ((data[6] & 0x3) != 0x3) & (data[7] != 0xFF) ) {
                            dev->stream[i]->pid_value = (float)(((((uint32_t)(data[6] & 0x3) << 8) | (uint32_t)(data[7])) * (float)0.00390625) - 2);
                            process_change(dev->stream[i]);
                        }
                    }
                    break;
                #endif

                #ifdef SNIFF_LATERAL_ACCELERATION_PID
                case 0x180:
                    /* Lateral Acceleration */
                    if( dev->stream[i]->pid_uuid == SNIFF_LATERAL_ACCELERATION_UUID ) {
                        if( ((data[2] & 0x3) != 0x3) & (data[3] != 0xFF) ) {
                            dev->stream[i]->pid_value = (float)(((((uint32_t)(data[2] & 0x3) << 8) | (uint32_t)(data[3])) * (float)0.00390625) - 2);
                            process_change(dev->stream[i]);
                        }
                    }
                    break;
                #endif

                #endif

                default:
                    break;
			}
    	}
    }
}

void CAN_Sniffer_tick( void )
{
    sniffer_tick++;
}

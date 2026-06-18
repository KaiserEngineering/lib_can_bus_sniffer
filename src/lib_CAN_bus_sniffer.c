/* Author : Matthew Kaiser */
/* @Description: Library to sniff proprietary data from the CAN bus of a vehicle and  *
 * relate it to standard OBD-II parameters (PIDs). The library will require access to *
 * the hardware peripheral's filter configuration and the main loop shall pass the    *
 * relevant packets to the library when received. The library will update the PID     *
 * pointer value as it receives new data.                                             */
#include "lib_CAN_bus_sniffer.h"
#include "lib_CAN_bus_sniffer_default_json.h"
#include "cjson_shared.h"
#include <string.h>
#include <stdlib.h>

/* Number of filters that are supported for the library, this can be different than *
 * the number of filters supported by the hardware.                                 */
#ifndef MAX_CAN_FILTERS
#define MAX_CAN_FILTERS 25
#endif
uint16_t active_filters[MAX_CAN_FILTERS];
#define RESERVED_FILTER 0xFFFF

uint32_t sniffer_tick = 0;

static uint8_t parse_u32_json( cJSON *item, uint32_t *value );
static uint8_t parse_mode_json( cJSON *item, uint8_t *mode );
static uint32_t pid_uuid_from_entry( cJSON *entry );
static cJSON *json_item( cJSON *object, const char *first_key, const char *second_key );
static uint8_t json_string_equals( cJSON *item, const char *value );
static uint16_t dbc_motorola_start_to_msb0( uint16_t start_bit );
static void load_definition_entry( PCAN_SNIFFER_PACKET_MANAGER dev, cJSON *entry, cJSON *parent_id );
static PCAN_SNIFF_SIGNAL_DEFINITION find_definition( PCAN_SNIFFER_PACKET_MANAGER dev, uint32_t pid_uuid );
static uint8_t stream_uses_arbitration_id( PCAN_SNIFFER_PACKET_MANAGER dev, uint16_t arbitration_id );
static void clear_streams( PCAN_SNIFFER_PACKET_MANAGER dev );
static void config_filter( PCAN_SNIFFER_PACKET_MANAGER dev, uint16_t id, uint8_t enable );
static uint32_t extract_signal_raw( const uint8_t *data, PCAN_SNIFF_SIGNAL_DEFINITION definition );
static float signal_raw_to_float( uint32_t raw, PCAN_SNIFF_SIGNAL_DEFINITION definition );

/* Parse decimal JSON numbers or hex strings like "0F8" / "0802". */
static uint8_t parse_u32_json( cJSON *item, uint32_t *value )
{
    char *end = NULL;

    if( (item == NULL) || (value == NULL) )
        return 0;

    if( cJSON_IsNumber(item) )
    {
        *value = (uint32_t)item->valuedouble;
        return 1;
    }

    if( cJSON_IsString(item) && (item->valuestring != NULL) )
    {
        *value = (uint32_t)strtoul(item->valuestring, &end, 16);
        return (end != item->valuestring) ? 1U : 0U;
    }

    return 0;
}

/* Convert JSON mode names into the same numeric namespace used by lib_pid. */
static uint8_t parse_mode_json( cJSON *item, uint8_t *mode )
{
    uint32_t value = 0;

    if( (item == NULL) || (mode == NULL) )
        return 0;

    if( cJSON_IsString(item) && (item->valuestring != NULL) )
    {
        if( strcmp(item->valuestring, "MODE1") == 0 )
        {
            *mode = MODE1;
            return 1;
        }

        if( strcmp(item->valuestring, "MODE2") == 0 )
        {
            *mode = MODE2;
            return 1;
        }

        if( strcmp(item->valuestring, "MODE22") == 0 )
        {
            *mode = MODE22;
            return 1;
        }

        if( strcmp(item->valuestring, "SNIFF") == 0 )
        {
            *mode = SNIFF;
            return 1;
        }

        if( strcmp(item->valuestring, "CALC1") == 0 )
        {
            *mode = CALC1;
            return 1;
        }
    }

    if( parse_u32_json(item, &value) )
    {
        *mode = (uint8_t)value;
        return 1;
    }

    return 0;
}

/* Build the shared PID identity used by lib_pid: pid_uuid = (mode << 16) | pid. */
static uint32_t pid_uuid_from_entry( cJSON *entry )
{
    uint8_t mode = 0;
    uint32_t pid = 0;
    uint32_t pid_uuid = 0;

    if( parse_u32_json(cJSON_GetObjectItem(entry, "pid_uuid"), &pid_uuid) )
        return pid_uuid;

    if( !parse_mode_json(cJSON_GetObjectItem(entry, "mode"), &mode) )
        return PID_UNASSIGNED;

    if( !parse_u32_json(cJSON_GetObjectItem(entry, "pid"), &pid) )
        return PID_UNASSIGNED;

    return ((uint32_t)mode << 16) | (pid & 0xFFFFU);
}

/* Accept both current camelCase keys and older snake_case keys while the JSON
 * format is still settling. */
static cJSON *json_item( cJSON *object, const char *first_key, const char *second_key )
{
    cJSON *item = cJSON_GetObjectItem(object, first_key);

    if( (item == NULL) && (second_key != NULL) )
        item = cJSON_GetObjectItem(object, second_key);

    return item;
}

static uint8_t json_string_equals( cJSON *item, const char *value )
{
    return cJSON_IsString(item) && (item->valuestring != NULL) && (strcmp(item->valuestring, value) == 0);
}

/* Vector DBC Motorola (@0) start bits are numbered inside each byte in the
 * opposite direction from this library's linear MSB0 extractor. Convert once
 * during JSON load so runtime decode can stay simple.
 *
 * Example: DBC 17|10@0+ becomes internal start bit 22.
 */
static uint16_t dbc_motorola_start_to_msb0( uint16_t start_bit )
{
    return (uint16_t)((start_bit & 0xFFF8U) + (7U - (start_bit & 0x0007U)));
}

/* Convert one JSON entry into the compact runtime definition table. Metadata like
 * label, units, min/max, and decimals is intentionally ignored here; other layers
 * can pass the original JSON downstream when they need display metadata. */
static void load_definition_entry( PCAN_SNIFFER_PACKET_MANAGER dev, cJSON *entry, cJSON *parent_id )
{
    uint32_t arbitration_id = 0;
    uint32_t start_bit = 0;
    uint32_t bit_len = 0;
    cJSON *byte_order = NULL;
    cJSON *scale = NULL;
    cJSON *offset = NULL;
    cJSON *is_signed = NULL;
    cJSON *invalid_raw_min = NULL;
    uint32_t pid_uuid = PID_UNASSIGNED;

    if( (dev == NULL) || (entry == NULL) || (dev->num_definitions >= CAN_SNIFF_MAX_DEFINITIONS) )
        return;

    if( !parse_u32_json(json_item(entry, "id", NULL), &arbitration_id) )
    {
        if( !parse_u32_json(parent_id, &arbitration_id) )
            return;
    }

    if( !parse_u32_json(json_item(entry, "startBit", "start_bit"), &start_bit) )
        return;

    if( !parse_u32_json(json_item(entry, "bitLen", "length"), &bit_len) )
        return;

    pid_uuid = pid_uuid_from_entry(entry);

    if( pid_uuid == PID_UNASSIGNED )
        return;

    byte_order = json_item(entry, "byteOrder", "byte_order");
    scale = cJSON_GetObjectItem(entry, "scale");
    offset = cJSON_GetObjectItem(entry, "offset");
    is_signed = cJSON_GetObjectItem(entry, "signed");
    invalid_raw_min = json_item(entry, "invalidRawMin", "invalid_raw_min");

    PCAN_SNIFF_SIGNAL_DEFINITION definition = &dev->definition[dev->num_definitions];

    definition->pid_uuid = pid_uuid;
    definition->arbitration_id = (uint16_t)arbitration_id;
    definition->start_bit = (uint16_t)start_bit;
    definition->length = (uint8_t)bit_len;
    definition->byte_order = CAN_SNIFF_BYTE_ORDER_MSB0;
    definition->is_signed = cJSON_IsTrue(is_signed);
    definition->scale = cJSON_IsNumber(scale) ? (float)scale->valuedouble : 1.0f;
    definition->offset = cJSON_IsNumber(offset) ? (float)offset->valuedouble : 0.0f;
    definition->has_invalid_raw_min = cJSON_IsNumber(invalid_raw_min);
    definition->invalid_raw_min = cJSON_IsNumber(invalid_raw_min) ? (uint32_t)invalid_raw_min->valuedouble : 0;

    if( json_string_equals(byte_order, "motorola") || json_string_equals(byte_order, "@0") )
    {
        definition->start_bit = dbc_motorola_start_to_msb0((uint16_t)start_bit);
        definition->byte_order = CAN_SNIFF_BYTE_ORDER_MSB0;
    }
    else if( json_string_equals(byte_order, "intel") || json_string_equals(byte_order, "lsb0") || json_string_equals(byte_order, "@1") )
    {
        definition->byte_order = CAN_SNIFF_BYTE_ORDER_LSB0;
    }

    dev->num_definitions++;
}

/* Look up a loaded definition by PID UUID. Used for support checks and when a PID
 * is first added to the active stream. */
static PCAN_SNIFF_SIGNAL_DEFINITION find_definition( PCAN_SNIFFER_PACKET_MANAGER dev, uint32_t pid_uuid )
{
    if( dev == NULL )
        return NULL;

    for( uint8_t i = 0; i < dev->num_definitions; i++ )
    {
        if( dev->definition[i].pid_uuid == pid_uuid )
            return &dev->definition[i];
    }

    return NULL;
}

/* Hardware filters are per CAN arbitration ID, not per PID. This tells remove
 * logic whether another active PID still needs the same CAN ID. */
static uint8_t stream_uses_arbitration_id( PCAN_SNIFFER_PACKET_MANAGER dev, uint16_t arbitration_id )
{
    for( uint8_t i = 0; i < dev->num_pids; i++ )
    {
        if( dev->stream_definition[i] == NULL )
            continue;

        if( dev->stream_definition[i]->arbitration_id == arbitration_id )
            return 1;
    }

    return 0;
}

/* Clear active streaming PIDs and remove filters they owned. Definitions remain
 * separate and are cleared by CAN_Sniffer_Load_JSON() before reloading. */
static void clear_streams( PCAN_SNIFFER_PACKET_MANAGER dev )
{
    if( dev == NULL )
        return;

    for( uint8_t i = 0; i < dev->num_pids; i++ )
    {
        if( dev->stream_definition[i] != NULL )
            config_filter(dev, dev->stream_definition[i]->arbitration_id, 0);
    }

    for( uint8_t i = 0; i < NUM_CAN_SNIFF; i++ )
    {
        dev->stream[i] = NULL;
        dev->stream_definition[i] = NULL;
    }

    dev->num_pids = 0;
}

/* MSB0 matches the bit numbering used by the current placeholder definitions:
 * bit 0 is the most-significant bit of byte 0. */
static uint8_t get_bit_msb0( const uint8_t *data, uint16_t bit )
{
    uint16_t byte_index = bit / 8U;
    uint8_t bit_index = 7U - (uint8_t)(bit % 8U);

    return (data[byte_index] >> bit_index) & 0x01U;
}

/* LSB0 is available for definitions that number bit 0 as the least-significant
 * bit of byte 0. */
static uint8_t get_bit_lsb0( const uint8_t *data, uint16_t bit )
{
    uint16_t byte_index = bit / 8U;
    uint8_t bit_index = (uint8_t)(bit % 8U);

    return (data[byte_index] >> bit_index) & 0x01U;
}

/* Extract the raw integer signal from an 8-byte CAN payload. This is generic and
 * a little slower than hand-written masks, but keeps decode driven by JSON. */
static uint32_t extract_signal_raw( const uint8_t *data, PCAN_SNIFF_SIGNAL_DEFINITION definition )
{
    uint32_t raw = 0;

    if( definition->byte_order == CAN_SNIFF_BYTE_ORDER_LSB0 )
    {
        for( uint8_t bit = 0; bit < definition->length; bit++ )
            raw |= ((uint32_t)get_bit_lsb0(data, definition->start_bit + bit) << bit);
    }
    else
    {
        for( uint8_t bit = 0; bit < definition->length; bit++ )
            raw = (raw << 1) | get_bit_msb0(data, definition->start_bit + bit);
    }

    return raw;
}

/* Apply signed conversion, scale, and offset after raw bit extraction. */
static float signal_raw_to_float( uint32_t raw, PCAN_SNIFF_SIGNAL_DEFINITION definition )
{
    int32_t signed_raw = (int32_t)raw;

    if( (definition->is_signed != 0U) && (definition->length > 0U) && (definition->length < 32U) )
    {
        uint32_t sign_bit = 1UL << (definition->length - 1U);
        if( (raw & sign_bit) != 0U )
            signed_raw = (int32_t)(raw | (~((1UL << definition->length) - 1UL)));
    }

    return ((float)signed_raw * definition->scale) + definition->offset;
}

/* Load sniffer definitions from JSON. The preferred format is a root array. The
 * object formats are retained for compatibility with earlier experiments. */
CAN_SNIFF_JSON_STATUS CAN_Sniffer_Load_JSON( PCAN_SNIFFER_PACKET_MANAGER dev, const char *json )
{
    if( (dev == NULL) || (json == NULL) )
        return CAN_SNIFF_JSON_ERROR;

    if( !cjson_shared_acquire() )
        return CAN_SNIFF_JSON_ERROR;

    clear_streams(dev);
    dev->num_definitions = 0;

    cJSON *root = cJSON_Parse(json);

    if( root == NULL )
    {
        cjson_shared_release();
        return CAN_SNIFF_JSON_ERROR;
    }

    if( cJSON_IsArray(root) )
    {
        cJSON *entry = NULL;
        cJSON_ArrayForEach(entry, root)
            load_definition_entry(dev, entry, NULL);
    }
    else
    {
        cJSON *messages = cJSON_GetObjectItem(root, "messages");

        if( cJSON_IsArray(messages) )
        {
            cJSON *message = NULL;
            cJSON_ArrayForEach(message, messages)
            {
                cJSON *id = cJSON_GetObjectItem(message, "id");
                cJSON *signals = cJSON_GetObjectItem(message, "signals");

                if( cJSON_IsArray(signals) )
                {
                    cJSON *signal = NULL;
                    cJSON_ArrayForEach(signal, signals)
                        load_definition_entry(dev, signal, id);
                }
                else
                {
                    load_definition_entry(dev, message, NULL);
                }
            }
        }
        else
        {
            cJSON *vehicle = NULL;
            cJSON_ArrayForEach(vehicle, root)
            {
                if( !cJSON_IsArray(vehicle) )
                    continue;

                cJSON *entry = NULL;
                cJSON_ArrayForEach(entry, vehicle)
                    load_definition_entry(dev, entry, NULL);
            }
        }
    }

    cJSON_Delete(root);
    cjson_shared_release();

    return (dev->num_definitions > 0U) ? CAN_SNIFF_JSON_OK : CAN_SNIFF_JSON_ERROR;
}

/* Initialize variables to a known state and verify the proper callbacks have been  *
 * assigned.                                                                        */
void CAN_Sniffer_Initialize( PCAN_SNIFFER_PACKET_MANAGER dev )
{
    dev->status = 0;

	/* Clear the PID count */
    dev->num_pids = 0;

    /* Clear the active filters */
    for( uint8_t i = 0; i < MAX_CAN_FILTERS; i++ )
    	active_filters[i] = RESERVED_FILTER;

    clear_streams(dev);

    dev->num_definitions = 0;
    (void)CAN_Sniffer_Load_JSON(dev, default_sniffer_json);

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

/* Verify that that the PID is supported by the active definition table. */
PID_SUPPORTED_STATUS CAN_Sniffer_PID_Supported( PCAN_SNIFFER_PACKET_MANAGER dev, PTR_PID_DATA pid )
{
    if( (dev == NULL) || (pid == NULL) )
        return PID_NOT_SUPPORTED;

    return (find_definition(dev, pid->pid_uuid) != NULL) ? PID_SUPPORTED : PID_NOT_SUPPORTED;
}

/* Add a PID to the packet manager to be streamed. This will return   *
 * @PID_SUPPORTED_STATUS to verify if the PID was or was not added.   *
 * Upon adding a supported PID, the library will request a hardware   *
 * filter if necessary (see add_filter)                               */
PID_SUPPORTED_STATUS CAN_Sniffer_Add_PID( PCAN_SNIFFER_PACKET_MANAGER dev, PTR_PID_DATA pid )
{
    if( (dev == NULL) || (pid == NULL) )
        return PID_NOT_SUPPORTED;

    PCAN_SNIFF_SIGNAL_DEFINITION definition = find_definition(dev, pid->pid_uuid);

    if( definition == NULL )
        return PID_NOT_SUPPORTED;

    for( uint8_t i = 0; i < dev->num_pids; i++ )
    {
        if( dev->stream[i] == pid )
            return PID_SUPPORTED;
    }

    if( dev->num_pids >= NUM_CAN_SNIFF )
        return PID_NOT_SUPPORTED;

    /* Activate one hardware filter for the CAN ID, then cache the definition so
     * packet decode does not need to search definition[] repeatedly. */
    config_filter(dev, definition->arbitration_id, 1);

    dev->stream[dev->num_pids] = pid;
    dev->stream_definition[dev->num_pids] = definition;
    dev->num_pids++;

	return PID_SUPPORTED;
}


PID_SUPPORTED_STATUS CAN_Sniffer_Remove_PID( PCAN_SNIFFER_PACKET_MANAGER dev, PTR_PID_DATA pid )
{
    if( (dev == NULL) || (pid == NULL) )
        return PID_NOT_SUPPORTED;

    PCAN_SNIFF_SIGNAL_DEFINITION definition = find_definition(dev, pid->pid_uuid);

    if( definition == NULL )
        return PID_NOT_SUPPORTED;

    /* Cycle through all the PIDs to find which one must be removed */
    for( uint8_t index = 0; index < dev->num_pids; index++ )
    {
        /* If found, pop that pointer reference */
        if( dev->stream[index] == pid )
        {
            for( uint8_t i = index; i + 1U < dev->num_pids; i++ )
            {
                dev->stream[i] = dev->stream[i + 1U];
                dev->stream_definition[i] = dev->stream_definition[i + 1U];
            }

            /* Remove the PID */
            dev->num_pids--;
            dev->stream[dev->num_pids] = NULL;
            dev->stream_definition[dev->num_pids] = NULL;

            if( stream_uses_arbitration_id(dev, definition->arbitration_id) == 0U )
                config_filter(dev, definition->arbitration_id, 0);

            return PID_SUPPORTED;
        }
    }

    return PID_NOT_SUPPORTED;
}

static void process_change( PTR_PID_DATA pid, float value )
{
	update_pid_data(pid, value, sniffer_tick);
}

void CAN_Sniffer_Add_Packet( PCAN_SNIFFER_PACKET_MANAGER dev, uint16_t arbitration_id, uint8_t* data )
{
	if( (dev == NULL) || (data == NULL) )
	    return;

	/* Only active stream entries are checked. Each entry already has its JSON
	 * definition cached, so the runtime path is: CAN ID match -> extract raw
	 * bits -> skip invalid sentinel -> scale -> update PID data. */
    for( uint8_t i = 0; i < dev->num_pids; i++ )
    {
    	if( dev->stream[i] != NULL )
    	{
            if( dev->stream_definition[i] == NULL )
                continue;

            if( dev->stream_definition[i]->arbitration_id != arbitration_id )
                continue;

            uint32_t raw = extract_signal_raw(data, dev->stream_definition[i]);

            if( (dev->stream_definition[i]->has_invalid_raw_min != 0U) && (raw >= dev->stream_definition[i]->invalid_raw_min) )
                continue;

            process_change(dev->stream[i], signal_raw_to_float(raw, dev->stream_definition[i]));
    	}
    }
}

void CAN_Sniffer_tick( void )
{
    sniffer_tick++;
}

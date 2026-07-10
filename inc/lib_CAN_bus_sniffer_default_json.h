/* Embedded fallback sniffer definitions.
 *
 * This header is intentionally simple: it lets the firmware ship with a default
 * JSON table even when no SD-card/file-system loader is available yet. The
 * source JSON lives in lib/lib_can_bus_sniffer/data/can_bus_sniffer.json; later
 * this file can be generated from that JSON instead of being hand-maintained.
 */
#ifndef LIB_CAN_BUS_SNIFFER_DEFAULT_JSON_H_
#define LIB_CAN_BUS_SNIFFER_DEFAULT_JSON_H_

static const char default_sniffer_json[] =
"["
    "{\"label\":\"RPM\",\"desc\":\"Engine Speed\",\"id\":\"090\",\"mode\":\"MODE1\",\"pid\":\"0C\",\"dataLen\":0,\"units\":[\"RPM\"],\"min\":[0],\"max\":[8000],\"decimals\":[0],\"formula\":\"CAN_RAW\",\"startBit\":35,\"bitLen\":12,\"byteOrder\":\"motorola\",\"scale\":2,\"offset\":0},"
    "{\"label\":\"APP\",\"desc\":\"Accel Pedal Pos\",\"id\":\"080\",\"mode\":\"MODE1\",\"pid\":\"5A\",\"dataLen\":0,\"units\":[\"PERCENT\"],\"min\":[0],\"max\":[100],\"decimals\":[0],\"formula\":\"CAN_RAW\",\"startBit\":1,\"bitLen\":10,\"byteOrder\":\"motorola\",\"scale\":0.1,\"offset\":0},"
    "{\"label\":\"Brake\",\"desc\":\"Brake Pedal Status\",\"id\":\"080\",\"mode\":\"SNIFF\",\"pid\":\"0802\",\"dataLen\":0,\"units\":[\"BOOLEAN\"],\"min\":[0],\"max\":[1],\"decimals\":[0],\"formula\":\"CAN_RAW\",\"startBit\":2,\"bitLen\":1,\"byteOrder\":\"motorola\",\"scale\":1,\"offset\":0},"
    "{\"label\":\"Reverse\",\"desc\":\"Reverse Status\",\"id\":\"080\",\"mode\":\"SNIFF\",\"pid\":\"0803\",\"dataLen\":0,\"units\":[\"BOOLEAN\"],\"min\":[0],\"max\":[1],\"decimals\":[0],\"formula\":\"CAN_RAW\",\"startBit\":5,\"bitLen\":1,\"byteOrder\":\"motorola\",\"scale\":1,\"offset\":0},"
    "{\"label\":\"Oil Temp\",\"desc\":\"Oil Temp\",\"id\":\"0F8\",\"mode\":\"MODE1\",\"pid\":\"5C\",\"dataLen\":0,\"units\":[\"CELSIUS\",\"FAHRENHEIT\"],\"min\":[-40,-40],\"max\":[200,400],\"decimals\":[0,1],\"formula\":\"CAN_RAW\",\"startBit\":63,\"bitLen\":8,\"byteOrder\":\"motorola\",\"scale\":1,\"offset\":-60},"
    "{\"label\":\"PTU Temp\",\"desc\":\"PTU Temp [RS]\",\"id\":\"0F8\",\"mode\":\"SNIFF\",\"pid\":\"0F81\",\"dataLen\":0,\"units\":[\"CELSIUS\",\"FAHRENHEIT\"],\"min\":[-60,-76],\"max\":[195,383],\"decimals\":[0,1],\"formula\":\"CAN_RAW\",\"startBit\":63,\"bitLen\":8,\"byteOrder\":\"motorola\",\"scale\":1,\"offset\":-60},"
    "{\"label\":\"Boost\",\"desc\":\"Boost\",\"id\":\"0F8\",\"mode\":\"MODE1\",\"pid\":\"6F\",\"dataLen\":0,\"units\":[\"KPA\",\"PSI\",\"BAR\"],\"min\":[0,0,0],\"max\":[255,36,2.55],\"decimals\":[1,2,2],\"formula\":\"CAN_RAW\",\"startBit\":47,\"bitLen\":8,\"byteOrder\":\"motorola\",\"scale\":1,\"offset\":0},"
    "{\"label\":\"Illum\",\"desc\":\"Gauge Illum Level\",\"id\":\"0C8\",\"mode\":\"SNIFF\",\"pid\":\"01C8\",\"dataLen\":0,\"units\":[\"NONE\"],\"min\":[0],\"max\":[31],\"decimals\":[0],\"formula\":\"CAN_RAW\",\"startBit\":4,\"bitLen\":5,\"byteOrder\":\"motorola\",\"scale\":1,\"offset\":0},"
    "{\"label\":\"E-Brake\",\"desc\":\"Emergency Brake Status\",\"id\":\"0C8\",\"mode\":\"SNIFF\",\"pid\":\"0C82\",\"dataLen\":0,\"units\":[\"BOOLEAN\"],\"min\":[0],\"max\":[1],\"decimals\":[0],\"formula\":\"CAN_RAW\",\"startBit\":30,\"bitLen\":1,\"byteOrder\":\"motorola\",\"scale\":1,\"offset\":0},"
    "{\"label\":\"ON Button\",\"desc\":\"Cruise Control ON button\",\"id\":\"030\",\"mode\":\"SNIFF\",\"pid\":\"0301\",\"dataLen\":0,\"units\":[\"NONE\"],\"min\":[0],\"max\":[1],\"decimals\":[0],\"formula\":\"CAN_RAW\",\"startBit\":40,\"bitLen\":1,\"byteOrder\":\"motorola\",\"scale\":1,\"offset\":0},"
    "{\"label\":\"OFF Button\",\"desc\":\"Cruise Control OFF button\",\"id\":\"030\",\"mode\":\"SNIFF\",\"pid\":\"0302\",\"dataLen\":0,\"units\":[\"NONE\"],\"min\":[0],\"max\":[1],\"decimals\":[0],\"formula\":\"CAN_RAW\",\"startBit\":41,\"bitLen\":1,\"byteOrder\":\"motorola\",\"scale\":1,\"offset\":0},"
    "{\"label\":\"SET+ Button\",\"desc\":\"Cruise Control SET Plus Button\",\"id\":\"030\",\"mode\":\"SNIFF\",\"pid\":\"0303\",\"dataLen\":0,\"units\":[\"NONE\"],\"min\":[0],\"max\":[1],\"decimals\":[0],\"formula\":\"CAN_RAW\",\"startBit\":47,\"bitLen\":1,\"byteOrder\":\"motorola\",\"scale\":1,\"offset\":0},"
    "{\"label\":\"SET- Button\",\"desc\":\"Cruise Control SET Minus button\",\"id\":\"030\",\"mode\":\"SNIFF\",\"pid\":\"0304\",\"dataLen\":0,\"units\":[\"NONE\"],\"min\":[0],\"max\":[1],\"decimals\":[0],\"formula\":\"CAN_RAW\",\"startBit\":32,\"bitLen\":1,\"byteOrder\":\"motorola\",\"scale\":1,\"offset\":0},"
    "{\"label\":\"RES Button\",\"desc\":\"Cruise Control RES button\",\"id\":\"030\",\"mode\":\"SNIFF\",\"pid\":\"0305\",\"dataLen\":0,\"units\":[\"NONE\"],\"min\":[0],\"max\":[1],\"decimals\":[0],\"formula\":\"CAN_RAW\",\"startBit\":45,\"bitLen\":1,\"byteOrder\":\"motorola\",\"scale\":1,\"offset\":0},"
    "{\"label\":\"CAN Button\",\"desc\":\"Cruise Control CAN button\",\"id\":\"030\",\"mode\":\"SNIFF\",\"pid\":\"0306\",\"dataLen\":0,\"units\":[\"NONE\"],\"min\":[0],\"max\":[1],\"decimals\":[0],\"formula\":\"CAN_RAW\",\"startBit\":44,\"bitLen\":1,\"byteOrder\":\"motorola\",\"scale\":1,\"offset\":0},"
    "{\"label\":\"ECT\",\"desc\":\"Engine coolant temp\",\"id\":\"2F0\",\"mode\":\"MODE1\",\"pid\":\"05\",\"dataLen\":0,\"units\":[\"CELSIUS\",\"FAHRENHEIT\"],\"min\":[-60,-76],\"max\":[963,1765.4],\"decimals\":[0,1],\"formula\":\"CAN_RAW\",\"startBit\":33,\"bitLen\":10,\"byteOrder\":\"motorola\",\"scale\":1,\"offset\":-60},"
    "{\"label\":\"IAT\",\"desc\":\"Intake Air Temp\",\"id\":\"2F0\",\"mode\":\"MODE1\",\"pid\":\"0F\",\"dataLen\":0,\"units\":[\"CELSIUS\",\"FAHRENHEIT\"],\"min\":[-127,-196.6],\"max\":[128.75,263.75],\"decimals\":[2,2],\"formula\":\"CAN_RAW\",\"startBit\":49,\"bitLen\":10,\"byteOrder\":\"motorola\",\"scale\":0.25,\"offset\":-127},"
    "{\"label\":\"Lat Accel\",\"desc\":\"Lateral Acceleration\",\"id\":\"180\",\"mode\":\"SNIFF\",\"pid\":\"1802\",\"dataLen\":0,\"units\":[\"G\"],\"min\":[-2],\"max\":[2],\"decimals\":[2],\"formula\":\"CAN_RAW\",\"startBit\":17,\"bitLen\":10,\"byteOrder\":\"motorola\",\"scale\":0.00390625,\"offset\":-2,\"invalidRawMin\":1022},"
    "{\"label\":\"Long Accel\",\"desc\":\"Longitudinal Acceleration\",\"id\":\"160\",\"mode\":\"SNIFF\",\"pid\":\"1602\",\"dataLen\":0,\"units\":[\"G\"],\"min\":[-2],\"max\":[2],\"decimals\":[2],\"formula\":\"CAN_RAW\",\"startBit\":49,\"bitLen\":10,\"byteOrder\":\"motorola\",\"scale\":0.00390625,\"offset\":-2,\"invalidRawMin\":1022}"
"]";

#endif /* LIB_CAN_BUS_SNIFFER_DEFAULT_JSON_H_ */

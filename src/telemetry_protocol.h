/*
 * telemetry_protocol.h
 *
 *  Created on: 19 Apr 2018
 *      Author: Cl�ment Nussbaumer
 */

#ifndef TELEMETRY_TELEMETRY_PROTOCOL_H_
#define TELEMETRY_TELEMETRY_PROTOCOL_H_



enum DatagramPayloadType {
	//first byte: vehicle ID, second byte: payload type
    TELEMETRY = 0x10, EVENT = 0x11, CONTROL = 0x12, GPS = 0x13, TELEMETRY_ERT18 = 14
};


#define HEADER_PREAMBLE_FLAG 0x55
#define CONTROL_FLAG 0xFF

// Field sizes in bytes
#define SEQUENCE_NUMBER_SIZE 4
#define PAYLOAD_TYPE_SIZE 1

// Sections sizes in bytes
#define PREAMBLE_SIZE 4
#define HEADER_SIZE (SEQUENCE_NUMBER_SIZE + PAYLOAD_TYPE_SIZE)
#define CONTROL_FLAG_SIZE 1
#define CHECKSUM_SIZE 2
#define TOTAL_OVERHEAD (PREAMBLE_SIZE + HEADER_SIZE + CONTROL_FLAG_SIZE + CHECKSUM_SIZE)

#define SENSOR_PACKET_SIZE (36 + TOTAL_OVERHEAD)
#define GPS_PACKET_SIZE (21 + TOTAL_OVERHEAD)

#endif /* TELEMETRY_TELEMETRY_PROTOCOL_H_ */

/*
 * rplidar-a3-arduino library header
 *
 * Copyright 2018 (C) Bartosz Meglicki <meglickib@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. 
 *
 */

/*
 * This is asynchronous (non blocking) library for speed motor control
 * and communication with RPLidar A3. 
 * 
*/

#ifndef RPLIDAR_A3_H
#define RPLIDAR_A3_H

#include "Arduino.h"

#include <stdint.h>
#include <PID_v1.h>

enum {RPLidarPacketDataSize=132};

struct RPLidarPacket
{	
	uint32_t timestamp_us;//timestamp in microseconds
	uint8_t sequence;	  //0-255, wrap-around for checking if packets are consecutive
	uint8_t data[RPLidarPacketDataSize]; //raw measurement packet data
};

enum DescriptorMode : uint8_t {ModeSingle=0, ModeMultiple=1, ModeReserved1, ModeReserved2};

struct ResponseDescriptor
{
	uint32_t length;
	DescriptorMode mode;
	uint8_t type;
};

class RPLidar
{
	enum State {Idle, 
				HealthRequestTx, HealthRequestRx,
				ResetRequestTx, ResetRequestDelay,
				ScanRequestTx, ScanMeasurementRx,
				ScanTimeoutHealthRequestTx, ScanTimeoutHealthRequestRx,
				StopRequestTx, StopRequestDelay, 
				CommunicationError, HardwareFailure, CheckMotor};
				
	enum DescriptorState { DescriptorNotAwaiting, DescriptorStartFlag1, DescriptorStartFlag2, DescriptorBytes };
	enum MeasurementState { MeasurementNotAwaiting, MeasurementStartFlag1, MeasurementStartFlag2, MeasurementBytes };

	enum {TxBufferSize=256, RxBufferSize=256};
	
public:
	/* Setup */
	RPLidar(HardwareSerial &serial, int pwm_pin);
	void setup(int motor_rpm);	
	void start();
	
	
	/* Cyclic */
	bool processAvailable(RPLidarPacket *packet);
private:
	bool sendData();
	
	bool readDescriptor();
	bool readMeasurementData();
	void readData();
	bool readPending();

	/* States */
	void OnHealthRequestTx();
	void OnHealthRequestRx();
	void OnScanRequestTx();
	bool OnScanMeasurermentRx(RPLidarPacket *packet);

	bool isNewScan();
	void calculateMotorStats();
	bool applyMotorPID();

	/* Packets */
	uint8_t calculateCrc(const uint8_t *data, const int bytes);
	bool checkMeasurementCrc();
	
	void encodeGetHealth();
	void encodeExpressScan(uint8_t mode);

	void decodeResponseDescriptor();
	
	inline uint16_t decode_u16(const uint8_t *data) const
	{
		return (uint16_t)data[1] << 8 | data[0];
	}
	inline uint32_t decode_u32(const uint8_t *data) const
	{
		return (uint32_t) data[3] << 24 | data[2] << 16  | data[1] << 8 | data[0];
	}
private:
	/* Hardware */
	HardwareSerial &m_serial;
	const int m_pwm_pin;
		
	/* Packet & Decoding */ 
	State m_state;
	DescriptorState m_descriptor_state;
	ResponseDescriptor m_response_descriptor;
	MeasurementState m_measurement_state;
	
	uint8_t m_tx_buffer[TxBufferSize];
	uint8_t m_rx_buffer[RxBufferSize];

	int m_tx_bytes;
	int m_rx_bytes;
	int m_tx_sent;
	int m_rx_read;
	
	uint32_t m_packet_timestamp_us;
	uint32_t m_prev_packet_timestamp_us;
	uint8_t m_packet_sequence;
	bool m_packet_sequence_skip;
	uint16_t m_prev_angle_deg_q6;
	uint32_t m_prev_revolution_us;
	
	/* Motor control */
	PID m_motor_pid;
	double m_motor_actual_rpm;
	double m_motor_setpoint_rpm;
	double m_motor_pwm;
};

#endif

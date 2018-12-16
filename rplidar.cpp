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

/* The state machine:
 * 
 * NOTE - currently implemented only:
 * - Idle -> ScanRequestTx [start() function call)
 * - ScanRequestTx -> ScanMeasurementRx
 * - ScanMeasurementRx -> ScanMeasurementRx 
 * 
 * State transition                                 [triggering condition]
 * ----------------------------------------------------------------------
 *
 * Idle -> ScanRequestTx                            [user startScan call]
 * 
 * ScanRequestTx -> ScanMeasurementRx
 * 
*/

/*
 * Communication is Little Endian
 */

/* Request packet structure
 * 
 *  Field ||Start flag | Command | Payload Size | Payload | Checksum
 *  ------||-------------------------------------------------------- 
 *  bytes || 1 (0xA5)  |    1    |      1       |  0-255  |    1 
 * 
 */

/* Response descriptor packet structure
 * 
 *  Field ||Start flag 1 | Start flag 2 | Data Length | Send Mode | Data type
 *  ------||----------------------------------------------------------------- 
 *  bytes || 1 (0xA5)    | 1 (0x5A)     |  30 bits    | 2 bits    |    1 
 * 
 *  Data Length - bytes of single response packet
 *  Send Mode - 0/1/2/3 Single Response/Multiple Response/Reserved/Reserved
 *  Data Type - interpretation depends on request
 */
 
/* Response packet structure
 * 
 * Depends on request
 * 
 * For RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA  0x84 we have:
 * type=0x84
 * length=0x84 (132, the same as type)
 * mode=2 (ModeMultiple)
 * 
 * The packet is constructed as follows:
 * Byte  |  Value
 *   0   |  val >> 4 is sync bits 1, should equal to A ; checksum is (byte0 & 0xF) | (byte1 << 4)
 *   1   |  val >> 4 is sync bits 2, should equal to 5 ; checksum is (byte0 & 0xF) | (byte1 << 4)
 *  2,3  |  15 bits of starting angle q6 and sync bit (logically new scan)
 * 4-133 |  rest of the packet
*/


#include "rplidar.h"

enum Command {Stop=0x25, Reset=0x40, Scan=0x20, ExpressScan=0x82, ForceScan=0x21,
	  GetInfo=0x50, GetHealth=0x52, GetSampleRate=0x59};

enum Sizes {ResponseDescriptorBytes=7, GetHealthResponseBytes=7};

enum Byte {StartByte1=0xA5, StartByte2=0x5A, SyncBits1=0xA, SyncBits2=0x5};

enum Offsets {OffsetDescriptorDataLengthSendMode = 2, OffsetDescriptorDataType = 6,
			OffsetMeasurementCapsuledUltraCrc = 2, OffsetMeasurementCapsuledStartAngleSyncQ6=2};

enum Masks : uint32_t {MaskResponseDescriptorDataLength=0x3FFFFFFF, ShiftResponseDescriptorSendMode=30, MaskNewScan=1 << 15};

enum {SecondsPerMinute=60, MicrosPerSecond=1000000};

enum {PwmFrequency = 25000, UartBaudrate = 256000};

RPLidar::RPLidar(HardwareSerial &serial, int pwm_pin):
 m_serial(serial),
 m_pwm_pin(pwm_pin),
 m_state(Idle),
 m_descriptor_state(DescriptorNotAwaiting),
 m_measurement_state(MeasurementNotAwaiting),
 m_tx_bytes(0),
 m_rx_bytes(0),
 m_tx_sent(0),
 m_rx_read(0),
 m_packet_timestamp_us(0),
 m_prev_packet_timestamp_us(0),
 m_packet_sequence(0),
 m_packet_sequence_skip(0),
 m_prev_angle_deg_q6(20),
 m_prev_revolution_us(0),
 
 m_motor_pid(&m_motor_actual_rpm, &m_motor_pwm, &m_motor_setpoint_rpm, 1.0f, 0.05f, 0.0f, DIRECT	),

 m_motor_actual_rpm(250),
 m_motor_setpoint_rpm(250),
 m_motor_pwm(153) 
{

}

void RPLidar::setup(int motor_rpm)
{
  pinMode(m_pwm_pin, OUTPUT);
  analogWriteFrequency(m_pwm_pin, PwmFrequency);
  m_serial.begin(UartBaudrate, SERIAL_8N1);
  m_motor_setpoint_rpm=motor_rpm;
  m_motor_actual_rpm=motor_rpm;
  m_motor_pid.SetMode(AUTOMATIC);
  m_motor_pid.SetSampleTime(100);

  //temp
  analogWrite(m_pwm_pin, m_motor_pwm);
}

bool RPLidar::processAvailable(RPLidarPacket* packet)
{
	switch(m_state)
	{
		case Idle:
			break;
		case ScanRequestTx:
			OnScanRequestTx();
			break;
		case ScanMeasurementRx:
			return OnScanMeasurermentRx(packet);
			break;
	}

	return false;
}

void RPLidar::encodeExpressScan(uint8_t mode)
{
	m_tx_buffer[0]=StartByte1;
	m_tx_buffer[1]=ExpressScan;
	m_tx_buffer[2]=5; //payload_size
	m_tx_buffer[3]=mode;
	m_tx_buffer[4]=0; //reserved
	m_tx_buffer[5]=0; //reserved
	m_tx_buffer[6]=0; //reserved	
	m_tx_buffer[7]=0; //reserved
	m_tx_buffer[8]=calculateCrc(m_tx_buffer, 8); //crc
	m_tx_bytes=9;
	m_tx_sent=0;
}

void RPLidar::start()
{
	if(m_state != Idle)
		return;
	
	m_state = ScanRequestTx;	
	m_descriptor_state=DescriptorNotAwaiting;
	encodeExpressScan(3); //hardcoded mode
}

bool RPLidar::sendData()
{		
	int available=m_serial.availableForWrite();
	
	while(available && m_tx_sent < m_tx_bytes)
	{
		m_serial.write(m_tx_buffer[m_tx_sent]);
		--available;
		++m_tx_sent;
	}
	
	return m_tx_sent == m_tx_bytes;
}

void RPLidar::OnScanRequestTx()
{
	if(!sendData())
		return;
		
	m_state=ScanMeasurementRx;
	m_descriptor_state=DescriptorStartFlag1;
	m_rx_bytes=ResponseDescriptorBytes;
	m_rx_read=0;
}

bool RPLidar::OnScanMeasurermentRx(RPLidarPacket *packet)
{
	if(m_descriptor_state != DescriptorNotAwaiting)
	{
		if(!readDescriptor())
			return false; 

		//possibly sanity check descriptor (if matches expected)
		
		m_rx_bytes = m_response_descriptor.length;
		m_rx_read = 0;

		m_measurement_state=MeasurementStartFlag1;
	}		
		
	if(!readMeasurementData())
		return false;

	//we got data packet		
	
	if(isNewScan())
		m_packet_sequence_skip=true;

	calculateMotorStats();
	applyMotorPID();
			
	m_packet_sequence += m_packet_sequence_skip ? 2 : 1;
	packet->sequence=m_packet_sequence;
	
	packet->timestamp_us = m_packet_timestamp_us;
	
	memcpy(packet->data, m_rx_buffer, RPLidarPacketDataSize);
	
	m_rx_read=0;
	
	return true;
}

bool RPLidar::isNewScan()
{
	uint16_t start_angle_sync_q6=decode_u16(m_rx_buffer+OffsetMeasurementCapsuledStartAngleSyncQ6);
				
	return start_angle_sync_q6 & MaskNewScan;
}


/* Calcultating motor rpm needs some explanation
 * 
 * We keep track when packets cross 360 - 0 degrees boundrary.
 * We record the time between two such occurences and use it for rpm calculation.
 * Since each packet has multiple readings we try to estimate when crossing 0 happened
 * under assumption that readings are evenly spaced in time and degrees.
 * 
 * Note - the fractional part of angle is ignored currently
 */
void RPLidar::calculateMotorStats()
{
	uint16_t start_angle_sync_q6=decode_u16(m_rx_buffer+OffsetMeasurementCapsuledStartAngleSyncQ6);
	uint16_t angle_deg_q6=(start_angle_sync_q6 & 0x7FFF);

	if(angle_deg_q6 > m_prev_angle_deg_q6)
	{
		m_prev_angle_deg_q6=angle_deg_q6;
		return;
	}

	//packet time us is of the order of 10 ms (10 000 us, 14 bit)
	//360-m_prev_angle_deg is of the order of 20 degrees (q6 - 5 + 6, 11 bits)	
	uint32_t packet_time_us=m_packet_timestamp_us-m_prev_packet_timestamp_us;	
	uint32_t angle_to_zero_us=packet_time_us*((360 << 6) - m_prev_angle_deg_q6)/(angle_deg_q6 + (360 << 6) - m_prev_angle_deg_q6);
	uint32_t zero_deg_us=m_prev_packet_timestamp_us+angle_to_zero_us;
	
	if(m_prev_revolution_us != 0)
	{
		uint32_t revolution_us=zero_deg_us-m_prev_revolution_us;
		m_motor_actual_rpm = (double)SecondsPerMinute * MicrosPerSecond / revolution_us;
		//Serial.println(m_motor_actual_rpm);		
	}
	
	m_prev_angle_deg_q6=angle_deg_q6;	
	m_prev_revolution_us=zero_deg_us;
}

bool RPLidar::applyMotorPID()
{
	bool computed=m_motor_pid.Compute();
	if(computed)
		analogWrite(m_pwm_pin, m_motor_pwm);
	return computed;
}

bool RPLidar::readDescriptor()
{
	//add timeout
		
	while(m_serial.available())
	{
		uint8_t byte=m_serial.read();
		switch(m_descriptor_state)
		{
			case DescriptorStartFlag1:
				if(byte != StartByte1)
					continue;
				m_rx_buffer[0]=byte;
				m_rx_read = 1;
				m_descriptor_state = DescriptorStartFlag2;
				break;
			case DescriptorStartFlag2:
				if(byte != StartByte2)
				{
					m_rx_read=0;
					m_descriptor_state=DescriptorStartFlag1;
					continue;
				}
				m_rx_buffer[1]=byte;
				m_rx_read=2;
				m_descriptor_state=DescriptorBytes;
				break;
			case DescriptorBytes:
				m_rx_buffer[m_rx_read]=byte;
				++m_rx_read;
				if(m_rx_read < m_rx_bytes)
					continue;
				//got whole descriptor
				m_descriptor_state=DescriptorNotAwaiting;
				decodeResponseDescriptor();

				return true;
			default:
				break; //should never get here, 
		}
	}
	return false;
}

bool RPLidar::readMeasurementData()
{
	while(m_serial.available())
	{
		uint8_t byte=m_serial.read();
		switch(m_measurement_state)
		{
			case MeasurementStartFlag1:
				if(byte >> 4 != SyncBits1)
				{
					m_packet_sequence_skip=true;
					continue;					
				}
				m_prev_packet_timestamp_us=m_packet_timestamp_us;
				m_packet_timestamp_us=micros();
				m_rx_buffer[0]=byte;
				m_rx_read = 1;
				m_measurement_state = MeasurementStartFlag2;
				break;
			case MeasurementStartFlag2:
				if(byte >> 4 != SyncBits2)
				{
					m_rx_read=0;
					m_measurement_state=MeasurementStartFlag1;
					m_packet_sequence_skip=true;
					continue;
				}
				m_rx_buffer[1]=byte;
				m_rx_read=2;
				m_measurement_state=MeasurementBytes;
				break;
			case MeasurementBytes:
				m_rx_buffer[m_rx_read]=byte;
				++m_rx_read;
				if(m_rx_read < m_rx_bytes)
					continue;
				//got whole descriptor
				m_measurement_state=MeasurementStartFlag1;
				
				if(checkMeasurementCrc())
					return true;
				
				m_packet_sequence_skip=true;
				m_rx_read=0;
				break;
			default:
				break; //should never get here, 
		}
	}
	return false;
}

uint8_t RPLidar::calculateCrc(const uint8_t *data, const int bytes)
{
	uint8_t crc=0;
	for(int i=0;i<bytes;++i)
		crc ^= data[i];
	return crc;
}
bool RPLidar::checkMeasurementCrc()
{ 
	uint8_t crc_rcv = (m_rx_buffer[0] & 0xF) | (m_rx_buffer[1] << 4);
	uint8_t crc_calc = calculateCrc(m_rx_buffer + OffsetMeasurementCapsuledUltraCrc, m_rx_bytes - OffsetMeasurementCapsuledUltraCrc);
	
	return crc_rcv == crc_calc;
}


void RPLidar::decodeResponseDescriptor()
{
	m_response_descriptor.type= m_rx_buffer[OffsetDescriptorDataType];
	m_response_descriptor.length = decode_u32(m_rx_buffer+OffsetDescriptorDataLengthSendMode);
	m_response_descriptor.mode = (DescriptorMode) (m_response_descriptor.length >> ShiftResponseDescriptorSendMode);
	m_response_descriptor.length &= MaskResponseDescriptorDataLength;
}

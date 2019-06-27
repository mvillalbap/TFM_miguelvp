/*
 * pids.h
 *
 *  Created on: 29 abr. 2019
 *      Author: miguelvp
 */

#ifndef PIDS_H_
#define PIDS_H_

// Request available PIDs from any service
#define FIRST_PIDS					0x00
#define SECOND_PIDS					0x20
#define THIRD_PIDS					0x40
#define FOURTH_PIDS					0x60
#define FIFTH_PIDS					0x80
#define SIXTH_PIDS					0xA0


// Service 01 and 02
#define STATUS_DTC					0x01
#define FREEZE_DTC					0x02
#define FUEL_SYSTEM					0x03
#define ENGINE_LOAD					0x04
#define ENGINE_COOLANT_TEMP			0x05
#define ENGINE_RPM					0x0C
#define VEHICLE_SPEED				0x0D
#define INTAKE_AIR_TEMP				0x0F
#define MAF_RATE					0x10
#define THROTTLE_POS				0x11
#define POWER_TAKE_OFF				0x1E
#define TIME_SINCE_START			0x1F

#define DISTANCE_MALFUNCTION		0x21
#define EGR							0x2C
#define EGR_ERROR					0x2D

#define AMBIENT_AIR_TEMP			0x46
#define FUEL_TYPE					0x51
#define HYBRID_BATTERY_CHARGE		0x5B
#define ENGINE_FUEL_RATE			0x5E

#define ACTUAL_TORQUE				0x62
#define ENGINE_PERCENT_TORQUE		0x64
#define EXHAUST_PRESSURE			0x73
#define EXHAUST_GAS_TEMP_1			0x78
#define EXHAUST_GAS_TEMP_2			0x79
#define DPF_1						0x7A
#define DPF_2						0x7B
#define DPF_TEMP					0x7C
#define NOX_NTE						0x7D
#define PM_NTE						0x7E
#define ENGINE_RUNTIME				0x7F

#define NOX_SENSOR					0x83
#define PM_SENSOR					0x86
#define SCR							0x88
#define ENGINE_EXHAUST_FLOW_RATE	0x9E

#define NOX_SENSOR_CORRECTED		0xA1


// Service 09
#define MC_VIN						0x01
#define VIN_NUMBER					0x02
#define ECU_NAME					0x0A
#define MONITOR						0x0B
#define ENGINE_NUMBER				0x0D
#define EROTAN						0x0F


// Headers
#define LEN_HEADER_OBD				2
#define NEXT_NUMBER					3
#define NUM_BYTES					5
#define PAYLOAD						13


#define VIN_INIT_NUMBER				19
#define VIN_NEXT_LINE				7
#define VIN_MAX_NUMBER				28

// Correciones
#define CORRECCION_RPM		4
#define CORRECCION_AIR		-40

#endif /* PIDS_H_ */

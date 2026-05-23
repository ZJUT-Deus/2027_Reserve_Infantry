#ifndef IMU_H
#define IMU_H

#include <cstdint>
#include "main.h"
#include "fdcan.h"
#include <cstring> 
#include <string.h>
#include "bsp_uart.h"
#include "struct_typedef.h"

#define IMU_RX_BUFFER_SIZE 85
#define IMU_UART (&huart10)
#define ANGLE_TO_RAD 0.01745329251994329576923690768489f

extern uint8_t IMU_Rx_Bigyaw[IMU_RX_BUFFER_SIZE];
extern uint8_t IMU_Rx_yaw[IMU_RX_BUFFER_SIZE];

extern uint16_t g_decode_data_pos;  // �����������ĵ�ǰ���ݳ���
extern uint16_t g_uart_rx_cnt;      // ���ջ������ĵ�ǰ���ݳ���
//-------------�´�������----------------
/*------------------------------------------------MARCOS define------------------------------------------------*/
extern uint8_t g_uart_rx_buf[IMU_RX_BUFFER_SIZE];	//˫��������������
extern uint8_t g_decode_data[IMU_RX_BUFFER_SIZE]; 

#define PROTOCOL_FIRST_BYTE			(unsigned char)0x59
#define PROTOCOL_SECOND_BYTE		(unsigned char)0x53

#define PROTOCOL_HEADER_BYTE		(short)2

#define PROTOCOL_FIRST_BYTE_POS 		0
#define PROTOCOL_SECOND_BYTE_POS		1

#define PROTOCOL_TID_LEN				2
#define PROTOCOL_MIN_LEN				7	/*header(2B) + tid(2B) + len(1B) + CK1(1B) + CK2(1B)*/

#define CRC_CALC_START_POS				2
#define CRC_CALC_LEN(payload_len)		((payload_len) + 3)	/*3 = tid(2B) + len(1B)*/
#define PROTOCOL_CRC_DATA_POS(payload_len)			(CRC_CALC_START_POS + CRC_CALC_LEN(payload_len))

#define PAYLOAD_POS						5

#define SINGLE_DATA_BYTES				4

/*data id define*/
#define IMU_TEMP_ID				(unsigned char)0x01
#define ACCEL_ID				(unsigned char)0x10
#define ANGLE_ID				(unsigned char)0x20
#define MAGNETIC_ID				(unsigned char)0x30     /*Normalized value*/
#define RAW_MAGNETIC_ID			(unsigned char)0x31     /*original value*/
#define EULER_ID				(unsigned char)0x40
#define QUATERNION_ID			(unsigned char)0x41
#define UTC_ID					(unsigned char)0x50
#define SAMPLE_TIMESTAMP_ID		(unsigned char)0x51
#define DATA_READY_TIMESTAMP_ID	(unsigned char)0x52
#define LOCATION_ID				(unsigned char)0x60
#define SPEED_ID				(unsigned char)0x70

/*length for specific data id*/
#define IMU_TEMP_DATA_LEN				(unsigned char)2
#define ACCEL_DATA_LEN					(unsigned char)12
#define ANGLE_DATA_LEN					(unsigned char)12
#define MAGNETIC_DATA_LEN				(unsigned char)12
#define MAGNETIC_RAW_DATA_LEN			(unsigned char)12
#define EULER_DATA_LEN					(unsigned char)12
#define QUATERNION_DATA_LEN				(unsigned char)16
#define UTC_DATA_LEN					(unsigned char)11
#define SAMPLE_TIMESTAMP_DATA_LEN		(unsigned char)4
#define DATA_READY_TIMESTAMP_DATA_LEN	(unsigned char)4
#define LOCATION_DATA_LEN				(unsigned char)12
#define SPEED_DATA_LEN          		(unsigned char)12

/*factor for sensor data*/
#define NOT_MAG_DATA_FACTOR			0.000001f
#define MAG_RAW_DATA_FACTOR			0.001f

#define IMU_TEMP_DATA_FACTOR		0.01f

/*factor for gnss data*/
#define LONG_LAT_DATA_FACTOR		0.0000001
#define ALT_DATA_FACTOR				0.001f
#define SPEED_DATA_FACTOR			0.001f

typedef enum
{
	crc_err = -3,
	data_len_err = -2,
	para_err = -1,
	analysis_ok = 0,
	analysis_done = 1
}analysis_res_t;

#pragma pack(1)

typedef struct
{
	unsigned char header1;	/*0x59*/
	unsigned char header2;	/*0x53*/
	unsigned short tid;		/*1 -- 60000*/
	unsigned char len;		/*length of payload, 0 -- 255*/
}output_data_header_t;

typedef struct
{
	unsigned char data_id;
	unsigned char data_len;
}payload_data_t;

typedef struct
{
    unsigned int itow;
    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char miniute;
    unsigned char second;
}utc_data_t;

typedef struct
{
	float accel_x;			/*unit: m/s2*/
	float accel_y;
	float accel_z;

	float angle_x;			/*unit: �� (deg)/s*/
	float angle_y;
	float angle_z;

	float mag_x;			/*unit: ��һ��ֵ*/
	float mag_y;
	float mag_z;

	float raw_mag_x;		/*unit: mGauss*/
	float raw_mag_y;
	float raw_mag_z;

	float pitch;			/*unit: �� (deg)*/
	float roll;
	float yaw;

	float quaternion_data0;
	float quaternion_data1;
	float quaternion_data2;
	float quaternion_data3;

	double latitude;					/*unit: deg*/
	double longtidue;					/*unit: deg*/
	float altidue;						/*unit: m*/

	float vel_n;						/*unit: m/s */
	float vel_e;
	float vel_d;

	utc_data_t utc_data;                /*utc data*/

	unsigned int sample_timestamp;		/*unit: us*/
	unsigned int data_ready_timestamp;	/*unit: us*/

	float imu_temp;
}protocol_info_t;

#pragma pack()

//--------------------------------------
extern protocol_info_t g_output_info;


//-------------�´�������----------------
typedef struct
{
	uint8_t header;
	uint8_t tag;
	uint8_t slave_id;
	uint8_t reg;
	float data[3];
	uint16_t crc;
	uint8_t tail;

}normal_packet_t;


typedef struct
{
	uint8_t header;
	uint8_t tag;
	uint8_t slave_id;
	uint8_t reg;
	float data[4];
	uint16_t crc;
	uint8_t tail;

}normal_ext_packet_t;


typedef struct
{
	float accel[3];
	float gyro[3];
	float roll;
	float pitch;
	float yaw;
	float quaternion[4];

}dm_imu_t;

class IMU {

public:
    dm_imu_t imu_data;
    normal_packet_t normal_packet;
	normal_ext_packet_t ext_packet;
    uint8_t *Rx_Buffer_1;
	uint8_t *Rx_Buffer_2;
    uint16_t Rx_Buffer_Size;
    void imu_data_unpack();
    void init(UART_HandleTypeDef *huart,uint8_t *Rx_buf_1,uint8_t *Rx_buf_2,uint16_t Rx_buf_size);
	const dm_imu_t* get_imu_data_point();
	//-------------�´�������----------------
    protocol_info_t g_output_info;
    const protocol_info_t* get_imu_output_info_point();

	int analysis_data(unsigned char *data, short len);
	int get_signed_int(unsigned char *data);
	void clear_data(int clr_len);
	int calc_checksum(unsigned char *data, unsigned short len, unsigned short *checksum);
	unsigned char check_data_len_by_id(unsigned char id, unsigned char len, unsigned char *data);

};


#endif 


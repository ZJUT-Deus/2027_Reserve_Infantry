#include "imu.h"


uint8_t IMU_Rx_Bigyaw[IMU_RX_BUFFER_SIZE];
uint8_t IMU_Rx_yaw[IMU_RX_BUFFER_SIZE];
//-------------新代码试验----------------
uint8_t g_uart_rx_buf[IMU_RX_BUFFER_SIZE];	/*rx DMA buffer of uart2*/
uint16_t g_uart_rx_cnt = 0; /*reciede data length of uart2*/

uint8_t g_decode_data[IMU_RX_BUFFER_SIZE];	/*buffer for decoding*/
uint16_t g_decode_data_pos = 0;	/*bytes left in decode buffer*/
//--------------------------------------

protocol_info_t g_output_info;

//-------------新代码试验----------------
unsigned char IMU::check_data_len_by_id(unsigned char id, unsigned char len, unsigned char *data)
{
   	unsigned char ret = 0xff;

	switch(id)
	{
		case ACCEL_ID:
		{
			if(ACCEL_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.accel_x = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				g_output_info.accel_y = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_output_info.accel_z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case ANGLE_ID:
		{
			if(ANGLE_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.angle_x = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				g_output_info.angle_y = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_output_info.angle_z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case MAGNETIC_ID:
		{
			if(MAGNETIC_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.mag_x = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				g_output_info.mag_y = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_output_info.mag_z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case RAW_MAGNETIC_ID:
		{
			if(MAGNETIC_RAW_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.raw_mag_x = get_signed_int(data) * MAG_RAW_DATA_FACTOR;
				g_output_info.raw_mag_y = get_signed_int(data + SINGLE_DATA_BYTES) * MAG_RAW_DATA_FACTOR;
				g_output_info.raw_mag_z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * MAG_RAW_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case EULER_ID:
		{
			if(EULER_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.pitch = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				g_output_info.roll = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_output_info.yaw = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case QUATERNION_ID:
		{
			if(QUATERNION_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.quaternion_data0 = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				g_output_info.quaternion_data1 = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_output_info.quaternion_data2 = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
				g_output_info.quaternion_data3 = get_signed_int(data + SINGLE_DATA_BYTES * 3) * NOT_MAG_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case LOCATION_ID:
		{
			if(LOCATION_DATA_LEN == len)
			{
				int temp = 0;
				
				ret = (unsigned char)0x1;
				temp = *((int *)data);	/*solve the 'UNALIGNED' fault of Usage Faults*/
				g_output_info.latitude = temp * LONG_LAT_DATA_FACTOR;
				
				temp = *((int *)data + 1);
				g_output_info.longtidue = temp * LONG_LAT_DATA_FACTOR;
				
				temp = *((int *)data + 2);
				g_output_info.altidue = temp * ALT_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case SPEED_ID:
		{
			if(SPEED_DATA_LEN == len)
			{
				int temp = 0;
				
				ret = (unsigned char)0x1;
				
				temp = *((int *)data);	/*solve the 'UNALIGNED' fault of Usage Faults*/
				g_output_info.vel_n = temp * SPEED_DATA_FACTOR;
				
				temp = *((int *)data + 1);				
				g_output_info.vel_e = temp * SPEED_DATA_FACTOR;
				
				temp = *((int *)data + 2);				
				g_output_info.vel_d = temp * SPEED_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case IMU_TEMP_ID:
        {
			if(IMU_TEMP_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.imu_temp = *((short *)data) * IMU_TEMP_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
        }
        break;

		case UTC_ID:
        {
 			if(UTC_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				memcpy((unsigned char *)&g_output_info.utc_data.itow, data, len);
			}
			else
			{
				ret = (unsigned char)0x00;
			}
        }
        break;

		case SAMPLE_TIMESTAMP_ID:
		{
			if(SAMPLE_TIMESTAMP_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.sample_timestamp = *((unsigned int *)data);
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case DATA_READY_TIMESTAMP_ID:
		{
			if(DATA_READY_TIMESTAMP_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.data_ready_timestamp = *((unsigned int *)data);
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		default:
		break;
	}

	return ret;
}
int IMU::analysis_data(unsigned char *data, short len)
{
	unsigned short payload_len = 0;
	unsigned short check_sum = 0;
	unsigned short pos = 0;
	unsigned char ret = 0xff;
	unsigned char *temp = NULL;
	short i = len;
	
	output_data_header_t *header = NULL;
	payload_data_t *payload = NULL;

	if(NULL == data || 0 >= len)
	{
		return para_err;
	}

	if(len < PROTOCOL_MIN_LEN)
	{
		return data_len_err;
	}

	temp = data;
	
	while(i >= PROTOCOL_HEADER_BYTE)
	{
		/*judge protocol header*/		
		if(PROTOCOL_FIRST_BYTE == *temp && PROTOCOL_SECOND_BYTE == temp[1])
		{
			break;
		}
		else
		{
			temp++;
			i--;				
		}
	}	
	
	if(i < PROTOCOL_MIN_LEN)
	{
		clear_data(len - i);
		return data_len_err;	/*pack len err*/
	}	
	
	/*further check*/
	header = (output_data_header_t *)temp;
	payload_len = header->len;

	if(payload_len + PROTOCOL_MIN_LEN > i)
	{
		return 	data_len_err;
	}

	/*checksum*/
	calc_checksum(temp + CRC_CALC_START_POS, CRC_CALC_LEN(payload_len), &check_sum);
	if(check_sum != *((unsigned short *)(temp + PROTOCOL_CRC_DATA_POS(payload_len))))
	{
		clear_data(len - i + header->len + PROTOCOL_MIN_LEN);
		return crc_err;
	}

	/*analysis payload data*/
	pos = PAYLOAD_POS;

	while(payload_len > 0)
	{
		payload = (payload_data_t *)(temp + pos);
		ret = check_data_len_by_id(payload->data_id, payload->data_len, (unsigned char *)payload + 2);
		if((unsigned char)0x01 == ret)
		{
			pos += payload->data_len + sizeof(payload_data_t);
			payload_len -= payload->data_len + sizeof(payload_data_t);
		}
		else
		{
			pos++;
			payload_len--;
		}
	}

	clear_data(len - i + payload_len + PROTOCOL_MIN_LEN);
	
	return analysis_ok;
}

int IMU::get_signed_int(unsigned char *data)
{
	int temp = 0;

	temp = (int)((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);

	return temp;
}

void IMU::clear_data(int clr_len)
{
	memset(g_decode_data, 0, clr_len);	
	if(g_decode_data_pos > (unsigned int)clr_len)
	{
		g_decode_data_pos -= clr_len;
		memcpy(g_decode_data, g_decode_data + clr_len, g_decode_data_pos);
		memset(g_decode_data + g_decode_data_pos, 0, clr_len);
	}
	else
	{
		g_decode_data_pos = 0;
	}	
}

int IMU::calc_checksum(unsigned char *data, unsigned short len, unsigned short *checksum)
{
	unsigned char check_a = 0;
	unsigned char check_b = 0;
	unsigned short i;

	if(NULL == data || 0 == len || NULL == checksum)
	{
		return -1;
	}

	for(i = 0; i < len; i++)
	{
		check_a += data[i];
		check_b += check_a;
	}

	*checksum = ((unsigned short)(check_b << 8) | check_a);

	return 0;
}
void IMU::imu_data_unpack()
{
    	if(g_uart_rx_cnt > 0)
		{
			if(g_decode_data_pos + g_uart_rx_cnt <= IMU_RX_BUFFER_SIZE) 
        {
            memcpy(g_decode_data + g_decode_data_pos, g_uart_rx_buf, g_uart_rx_cnt);
            g_decode_data_pos += g_uart_rx_cnt;
        }
        else
        {
            g_decode_data_pos = 0;
        }
        g_uart_rx_cnt = 0;
		}		
		
		if(g_decode_data_pos > 0)
		{
			analysis_data(g_decode_data, g_decode_data_pos);
            
            g_decode_data_pos = 0;
		}

}    

//-------------新代码试验----------------

void IMU::init(UART_HandleTypeDef *huart,uint8_t *Rx_buf_1,uint8_t *Rx_buf_2,uint16_t Rx_buf_size)
{
    this->Rx_Buffer_1=Rx_buf_1;
    this->Rx_Buffer_2=Rx_buf_2;
    this->Rx_Buffer_Size=Rx_buf_size;
    UART_Init(huart, this->Rx_Buffer_1, this->Rx_Buffer_Size);
}

const dm_imu_t* IMU::get_imu_data_point()
{
    return &imu_data;
}

const protocol_info_t* IMU::get_imu_output_info_point()
{
    return &g_output_info;
}


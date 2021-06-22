#include "main.h"
#include "modbus_stm.h"

uint8_t rec_data[255] = {0};
uint8_t hold_reg_map[56]={0};
uint8_t inp_reg_map[66]={0};
int8_t write_reg_map[249]={-1};
uint8_t error_map[3]={0};
int status = 0;

static uint16_t MODBUS_CRC16( const unsigned char *buf, unsigned int len )
{
	uint16_t crc = 0xFFFF;
	unsigned int i = 0;
	char bit = 0;

	for( i = 0; i < len; i++ )
	{
		crc ^= buf[i];

		for( bit = 0; bit < 8; bit++ )
		{
			if( crc & 0x0001 )
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
			{
				crc >>= 1;
			}
		}
	}

	return crc;
}

int READ_HOLDING_REG(uint8_t Add_HI,uint8_t Add_LO,uint8_t Num_reg_HI,uint8_t Num_reg_LO)
{
	for(int k =0;k<255;k++)
		rec_data[k]=0;
	uint8_t buff[8] = {SLAVE_ADD,0x03,Add_HI,Add_LO,Num_reg_HI,Num_reg_LO,0,0};
	uint16_t crc;
	crc= MODBUS_CRC16(buff,6);
	buff[7] =  *((uint8_t*)&(crc)+1); //high byte 
	buff[6] =  *((uint8_t*)&(crc)+0); //low byte 
	uint16_t N=((unsigned int)Num_reg_HI << 8) + Num_reg_LO;

	Send_Data(buff,sizeof(buff));
	Receive_data( sizeof(buff)+ 2*N + 3);
	
	if(rec_data[1] == 0x83)//error code returned check error map array for exact error
	{
		error_map[0] = rec_data[2];
		return 2;
	}
	if(rec_data[1]==0x03)	// successfully transmitted command and successfully received data
	{
		uint16_t add_full = (Add_HI&0xFF00)|(Add_LO&0x00FF);
		add_full = add_full - 0x3FFF ;
		for(int i=0;i<N;i++,add_full++)
		{
			hold_reg_map[add_full] = rec_data[i+2];

		}
		return(1);
	}
	else
		{
			//HAL_UART_Transmit(&huart3,"error",6,500);
			return(0);
		}

}

int READ_INPUT_REG(uint8_t Add_HI,uint8_t Add_LO,uint8_t Num_reg_HI,uint8_t Num_reg_LO)
{
		uint8_t buff[8] = {SLAVE_ADD,0x04,Add_HI,Add_LO,Num_reg_HI,Num_reg_LO,0,0};
		uint16_t crc;
		crc= MODBUS_CRC16(buff,sizeof(buff)-2);
		buff[7] =  *((uint8_t*)&(crc)+1); //high byte
		buff[6] =  *((uint8_t*)&(crc)+0); //low byte
		for(int k =0;k<255;k++)
			rec_data[k]=0;
		uint16_t N=((unsigned int)Num_reg_HI << 8) + Num_reg_LO;//{Num_reg_HI|Num_reg_LO};


		Send_Data(buff,sizeof(buff));
		Receive_data(sizeof(buff)+ 2*N + 3);
		
		if(rec_data[1] == 0x84)
			{
				error_map[1] = rec_data[2];
				return 2;
			}
		if(rec_data[1]==0x04)
		{
			uint16_t add_full = (Add_HI&0xFF00)||(Add_HI&0x00FF);
			add_full = add_full - 0x3FE0 ;

			for(int i=1;i<N;i++)
			{
				inp_reg_map[add_full] = rec_data[i+2];
			}
			return(1);

		}
		else
			{
				return(0);
			}
}

int WRITE_SINGLE_REG(uint8_t Add_HI,uint8_t Add_LO,uint16_t value)
{
	uint8_t buff[8] = {SLAVE_ADD,0x06,Add_HI,Add_LO,(value&0xFF00),(value&0x00FF),0,0};
	uint16_t crc;
	crc= MODBUS_CRC16(buff,sizeof(buff)-2);
	buff[7] =  *((uint8_t*)&(crc)+1); //high byte
	buff[6] =  *((uint8_t*)&(crc)+0); //low byte
	for(int k =0;k<255;k++)
		rec_data[k]=0;
	Send_Data(buff,sizeof(buff));
	Receive_data(buff);
	
	if(rec_data[1] == 0x86)
		{
			error_map[2] = rec_data[2];
			return 2;
		}
	if(rec_data[1]==0x06)
	{
		if(rec_data[2]==buff[2]&&rec_data[3]==buff[3]&&rec_data[4]==buff[4]&&rec_data[5]==buff[5])
		{
			return 1;
		}
		else return 2;
	}
	else
		return 2;
}

int WRITE_MULTI_REG(uint8_t Add_HI, uint8_t Add_LO, uint8_t Num_of_reg_HI, uint8_t Num_of_reg_LO)
{
	for(int k =0;k<255;k++)
	rec_data[k]=0;
	uint8_t N =  2 * (((Num_of_reg_HI&0xFF00)|(Num_of_reg_LO&0x00FF))&(0x00FF));
	uint8_t buff[255]={0};
	buff[0]=SLAVE_ADD;
	buff[1]=0x10;
	buff[2]=Add_HI;
	buff[3]=Add_LO;
	buff[4]=Num_of_reg_HI;
	buff[5]=Num_of_reg_LO;
	buff[6]=N;
	for (int i=7;(i<N+7);i++)
	{
		buff[i]=write_reg_map[i-7];
	}


	uint16_t crc;
	crc= MODBUS_CRC16(buff,N+7);
	buff[N+8] =  *((uint8_t*)&(crc)+1); //high byte
	buff[N+7] =  *((uint8_t*)&(crc)+0); //low byte
	Send_Data(buff,N+9);
	Receive_data(sizeof(buff));


		if(rec_data[1] == 0x86)
			{
				error_map[3] = rec_data[2];
				for(int k =0;k<249;k++)
				write_reg_map[k]=-1;
				//free(buff);
				return 2;
			}
		if(rec_data[1]==0x10)
		{
			if(rec_data[2]==buff[2]&&rec_data[3]==buff[3]&&rec_data[4]==buff[4]&&rec_data[5]==buff[5])
			{
				for(int k =0;k<249;k++)
				write_reg_map[k]=-1;

				return 1;
			}
			else
				{
				for(int k =0;k<249;k++)
				write_reg_map[k]=-1;

					return 2;
				}
		}
		else
		{
			for(int k =0;k<249;k++)
			write_reg_map[k]=-1;

			return 2;
		}
}

uint8_t CHANGE_STATE(uint8_t next_state)
{
	for(int k =0;k<255;k++)
		rec_data[k]=0;
	uint8_t buff[8] = {SLAVE_ADD,0x06,0x40,0x00,0x00,next_state,0,0};
	uint16_t crc;
	crc= MODBUS_CRC16(buff,6);
	buff[7] =  *((uint8_t*)&(crc)+1); //high byte 
	buff[6] =  *((uint8_t*)&(crc)+0); //low byte  
	
	Send_Data(buff,sizeof(buff));
	Receive_data(sizeof(buff));
	
	if(rec_data[1] == 0x86)
		{
			error_map[2] = rec_data[2];
			return 2;
		}
	if(rec_data[1]==0x06)
	{
		if(rec_data[2]==buff[2]&&rec_data[3]==buff[3]&&rec_data[4]==buff[4]&&rec_data[5]==buff[5])
		{
			return 1;
		}
		else return 2;
	}
	else
		return 2;
}
	
	
int GET_STATE()
{
	uint8_t	current_state;
	READ_HOLDING_REG(0x40,0x00,0x00,0x00);
	current_state = hold_reg_map[1];
	return current_state;
}
	
void Send_Data(uint8_t * buff, uint8_t size)
{
	
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_Delay(24);//Silence on channel required for 24ms for baud rate 57600
	//HAL_UART_Transmit(&huart1,buff, sizeof(buff),500);
	HAL_UART_Transmit(&huart2,buff, size,5000);
	HAL_Delay(24);//Silence on channel required for 24ms for baud rate 57600
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);
	
}
	
void Receive_data(uint8_t size)
{
		HAL_UART_Receive (&huart2, rec_data, size , 500);
}
	
	
	
	
	
	
	
	
	
	

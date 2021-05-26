/*
 * ModBus.c
 *
 *  Created on: Feb 20, 2021
 *      Author: phung
 */

#include "main.h"

extern UART_HandleTypeDef huart6;

extern volatile uint8_t DataPos;

uint16_t MBRegisterCount(void)
{
	return (data_in[5]|(data_in[4]<<8));
}

void AppendDatatoMBRegister(uint16_t StAddr,uint16_t count, volatile uint16_t **inreg, volatile uint8_t *outreg)
{

	for (uint8_t c=0; c<count; c++)
	{
		*(outreg+3+c*2) = (uint8_t)   (*(inreg[StAddr+c]) >> 8) ;	//MSB IN HIGHER BYTE
		*(outreg+3+(c*2+1)) = (uint8_t) (*(inreg[StAddr+c]));	//LSB IN LOWER BYTE
	}
}

uint16_t MBStartAddress(void)	//Return requested start address
{
	return (data_in[3]|(data_in[2]<<8));
}

void MBSendData(uint8_t count)	//Send final data over UART
{

	HAL_UART_Transmit(&huart6, (uint8_t*)data_in, count, 100);
}

void AppendCRCtoMBRegister(uint8_t packtop)	//crc is calculated from slave id to last data byte
{
	uint16_t crcvalue = 0;
	crcvalue = CRC16((uint8_t*)data_in,packtop + 1);
	data_in[packtop+1] =(uint8_t)(crcvalue);			//lower byte at higher register
	data_in[packtop+2] =(uint8_t)(crcvalue>>8);		//higher byte at lower register

	ResponseFrameSize = packtop + 3;
}

void MBException(uint8_t exceptionCode)	//Exception code
{
	data_in[1]|=0x80; //setting MSB of the function code (the exception flag)
	data_in[2]=exceptionCode; //Exceptioncode. Also the last byte containing dat
	uint8_t packtop = 2;	// 3 bytes to send. No crc calculation.
	AppendCRCtoMBRegister(packtop);
	MBSendData(ResponseFrameSize);
}
void MBProcessRegisters(uint8_t fcCode)
{
	//First check what is the count of registers requested
	uint8_t RegCount = (uint8_t)MBRegisterCount();

	//| SLAVE_ID | FUNCTION_CODE | RETURN BYTES COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE            | N*2 BYTES | 2 BYTES |
	//So our final requested data should fit in above 256 size, so data should be max 256-6 bytes
	//As a safeguard we are also checking with maximum limits of query as per modbus function (m584 controller)
	if((RegCount >= 1u) &
		(RegCount*2 <= ((sizeof(data_in)/sizeof(data_in[0])) - 5u)) &
		(RegCount <= fc3_HoldingRegMax))
	{
		//to check if the requested start and end addresses are available in out controller
		//Get to know the starting address of the requested data
		uint16_t StAddress = MBStartAddress();
		uint16_t EndAddress = StAddress + RegCount - 1;

		//We will simply check if the end address is inside the size of our holding register
		if(EndAddress<=(sizeof(test))/sizeof(test[0]))
		{
			//Process the request
			data_in[2]=(uint8_t)(RegCount*2);	//fill the byte count in the data array
			AppendDatatoMBRegister(StAddress,RegCount,test,data_in);	//fill data in the data register
			AppendCRCtoMBRegister(2+RegCount*2);
			MBSendData(ResponseFrameSize);
		}
		else
		{
			MBException(0x02);//Exception code 02 = ILLEGAL DATA ADDRESS
		}
	}
	else
	{
		MBException(0x03);//Exception code 03 = ILLEGAL DATA VALUE
	}
}


void MBProcessBits(uint8_t *InArr, uint16_t InArrSize)
{
	//First check what is the count of bits requested
	uint16_t BitCount = MBRegisterCount();
	//As a safeguard we are also checking with maximum limits of query as per modbus function (m584 controller)

	uint8_t ByteCount = ((BitCount<=8u)
								?1u
								:((BitCount%8u==0)
									?(BitCount/8u)
									:((BitCount/8u)+1u)));
	//| SLAVE_ID | FUNCTION_CODE | RETURN BYTES COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE            | N BYTES   | 2 BYTES |
	if ((BitCount >=1u) &
		(BitCount <= 2000u))
	{
		//to check if the requested start and end addresses are available in out controller
		//As an example we configure 50 holding registers = 100 bytes of data array HoldingRegister
		//Get to know the starting address of the requested data
		uint16_t StAddress = MBStartAddress();				//start coil address
		uint16_t EndAddress = StAddress + BitCount - 1;	//end coil address

		if(EndAddress <=((InArrSize*8)-1))
		{
			//Process the request
			data_in[2]=(uint8_t)ByteCount;	//fill the byte count in the data array


			//Ex. if master request 4 coil statuses, this means that 1 register is response.
			//We need to clear the remaining 4 bits of the data if there is any.
			//Else there will be error
			uint8_t regtoclear = ((BitCount<8u)
										?3u
										:((BitCount%8u>0)
												?((uint8_t) (BitCount/8u+3u))
												:0));
			//clearing last byte of response array
			if (regtoclear>0) data_in[(uint8_t)regtoclear]= 0x00;

			AppendBitsToRegisters(StAddress,BitCount,InArr,data_in);
			AppendCRCtoMBRegister(2 + ByteCount);
			MBSendData(ResponseFrameSize);
		}
		else
		{
			MBException(0x02);//Exception code 02 = ILLEGAL DATA ADDRESS
		}
	}
	else
	{
		MBException(0x03);//Exception code 03 = ILLEGAL DATA VALUE
	}
}


void AppendBitsToRegisters(uint16_t StAddr, uint16_t count, uint8_t *inreg, volatile uint8_t *outreg)
{
	for (uint16_t c=0;c<count;c++)
	{
		if(*(inreg+((StAddr+c)/8u)) & (1<<((StAddr+c)-(((StAddr+c)/8u)*8u))))	//if in outreg array, bit is 1?
		{
			*(outreg+3+(c/8))|=(1<<(c-((c/8)*8)));	//then set bit 1 in target array
		} else *(outreg+3+(c/8))&=~(1<<(c-((c/8)*8)));	//else clear the bit in target array
	}
}


void WriteToRegister(volatile uint16_t** arr){

		ResponseFrameSize = DataPos;

		uint16_t temp_value = data_in[5]|(data_in[4]<<8);

		uint16_t temp_addr = data_in[3]|((data_in[2])<<8);

		if( (temp_value >= 0x0000) && (temp_value <= 0xffff))
		{

			if(temp_addr >= 0 && temp_addr <= HoldingRegSize)
			{
				*arr[temp_addr] = temp_value;
				MBSendData(ResponseFrameSize);

			}
			else
			{
				MBException(0x02);//Exception code 02 = ILLEGAL DATA ADDRESS
			}
		}
		else
		{
			MBException(0x03);//Exception code 03 = ILLEGAL DATA VALUE
		}

}

void WriteToMultilRegisters(volatile uint16_t** arr){

		ResponseFrameSize = 8;

		uint16_t length = sizeof(data_in)/sizeof(data_in[0]);

		uint16_t startAdr, quantityReg;

		startAdr = data_in[3]|(data_in[2]<<8);

		quantityReg = data_in[5]|(data_in[4]<<8);

		uint16_t value[quantityReg];

		if ((0x0001<=quantityReg)&&(quantityReg <= 500))
		{
			if ((startAdr>=0) && (startAdr < length))
			{
				for (uint8_t i = 0; i < quantityReg; i++)
				{
					value[i] = data_in[7+i*2+1]|(data_in[7+i*2]<<8);

					(*arr[startAdr +i]) = (uint16_t)(value[i]);
				}

				uint16_t crcvalue = 0;
				crcvalue = CRC16((uint8_t*)data_in, 6);

				data_in[6] = (uint8_t)(crcvalue);
				data_in[7] = (uint8_t)(crcvalue>>8);

				MBSendData(ResponseFrameSize);

			}
			else
			{
				MBException(0x02);
			}

		}
		else
		{
			MBException(0x03);
		}

}





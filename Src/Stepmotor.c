/*
 * Stepmotor.c
 *
 *  Created on: Feb 19, 2021
 *      Author: phung
 */

#include "main.h"

extern stepMotor_typeDef stepMotor1, stepMotor1;

void outport_motor(stepMotor_typeDef*motor, uint16_t pin1_value, uint16_t pin2_value, uint16_t pin3_value, uint16_t pin4_value)
{
	pin1_value = (pin1_value == 0)? 0:(motor->pin1);
	pin2_value = (pin2_value == 0)? 0:(motor->pin2);
	pin3_value = (pin3_value == 0)? 0:(motor->pin3);
	pin4_value = (pin4_value == 0)? 0:(motor->pin4);

	uint16_t temp_value = motor->Port->ODR & ((~motor->pin1) & (~motor->pin2) & (~motor->pin3) & (~motor->pin4));

	motor->Port->ODR = temp_value | (pin1_value | pin2_value |pin3_value |pin4_value);
}

void OneStep(stepMotor_typeDef* motor){
    if(motor->direction == CLOCKWISE)
    {
		switch(motor->temp_step)
		{
			case 0:
				outport_motor(motor,0,0,1,1);
			break;

			case 1:
				outport_motor(motor,0,1,1,0);
			break;

			case 2:
				outport_motor(motor,1,1,0,0);
			break;

			case 3:
				outport_motor(motor,1,0,0,1);
			break;
		}

    }else
    {
		switch(motor->temp_step)
		{
			case 0:
			outport_motor(motor,1,0,0,1);
			break;

			case 1:
			outport_motor(motor,1,1,0,0);
			break;

			case 2:
			outport_motor(motor,0,1,1,0);
			break;

			case 3:
			outport_motor(motor,0,0,1,1);
			break;
		}
    }

    motor->temp_step++;

    if(motor->temp_step > 3)
    {
    	motor->temp_step = 0;
    }

}






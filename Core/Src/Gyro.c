/*
 * Gyro.c
 *
 *  Created on: 16.07.2022
 *      Author: Hendr
 */
#include <stdio.h>
#include <string.h>
#include <gyro.h>
#include <stdbool.h>


bool InitialisiereGyro(){
	HAL_StatusTypeDef ret;
	uint8_t buf[15];

	//konfiguriere Control Register 1

	buf[0] = CTRL_REG1;
	buf[1] = 0b00001111; //Unused	Reset 0/1	SelfTest 0/1	Output Data Rate 011 für 100 Hz	Active Mode gewählt mit 11

	ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR_Gyro, buf, 2, HAL_MAX_DELAY);

	HAL_Delay(80);

	//lese Device Identifier

 	buf[0] = WHO_AM_I_Gyro_Reg;
	ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR_Gyro, buf, 1, 1000);
	HAL_Delay(80);
	if ( ret == HAL_OK ) {

		ret = HAL_I2C_Master_Receive(&hi2c1, ADDR_Gyro, buf, 1, HAL_MAX_DELAY); /*empfange den Device Identifier*/
		HAL_Delay(80);
		if ( ret == HAL_OK && buf[0] == GyroDeviceID) {
			return true;

		}else{
			strcpy((char*)buf, "INIT ERR Read");
			return false;
		}

	}else{
		strcpy((char*)buf, "INIT ERR Send");
		return false;
	}

}

void gyroWerteAuslesen (int16_t *x_axis, int16_t *y_axis, int16_t *z_axis){
	HAL_StatusTypeDef ret;



	uint8_t buf[12]; /*ein Buffer-Array*/
	buf[0] = Gyro_Status_REG;



	  /*hier ist die I2C-Übertragung*/

	  ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR_Gyro, buf, 1, HAL_MAX_DELAY); /*es wird ein byte gesendet, und zwar buf[0]*/
	  HAL_Delay(80);

	  if ( ret == HAL_OK ) {
		  //Burst-Read
		  ret = HAL_I2C_Master_Receive(&hi2c1, ADDR_Gyro, buf, 7, HAL_MAX_DELAY); /*empfange alle 6 Bytes für die Gyrowerte*/
		  HAL_Delay(80);
		if ( ret == HAL_OK ) {
			*x_axis = 0;
			*x_axis = (buf[1] << 8) | buf[2]; /* buf[1] enthält x_MSB; buf[2] enthält x_LSB. Deshalb wird das MSB in die Variable eingesetzt, um 8 bits verschoben, und dann LSB mit logischem Oder verknüpft*/
			*y_axis = 0;
			*y_axis = (buf[3] << 8) | buf[4];
			*z_axis = 0;
			*z_axis = (buf[5] << 8) | buf[6];

		}else{
			strcpy((char*)buf, "Error Read");
		}
	  }else{
		  strcpy((char*)buf, "Error Write");
	  }
}

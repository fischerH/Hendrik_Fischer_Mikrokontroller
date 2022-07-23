/*
 * Magnetometer.c
 *
 *  Created on: 20.07.2022
 *      Author: Hendr
 */

#include <stdio.h>
#include <string.h>
#include <Magnetometer.h>
#include <stdbool.h>


int InitialisiereMagnetometer(){
	HAL_StatusTypeDef ret;
	uint8_t buf[15];


	//Control Register 1
		//versetzt Sensor in Standby, da register bis auf Standy/Active Mode nur in Standby verändert werden kann
	buf[0] = FXOS8700CQ_CTRL_REG1;
	buf[1] = 0b00000000;
	ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR_Magnetometer, buf, 2, HAL_MAX_DELAY);

	HAL_Delay(80);
		//aktiviere Sensor und konfiguriere Control Register 1
	//Bit 7-6: auto-wake sample frequency; irrelevant (wähle 00)	Bit 5-3: Output data rate selection; wähle 010 für 200Hz mag only mode
	//bzw. 100 Hz hybrid mode	Bit 2: Inoise; wähle 0 für Normal mode	Bit 1: Fast Read Mode; wähle 0 für Normal Mode	Bit 0: wähle 1
	//um Sensor aus Standby zu holen und zu aktivieren
	buf[1] = 0b00010001;
	ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR_Magnetometer, buf, 2, HAL_MAX_DELAY);


	uint8_t MagnetometerDeviceIdentifier;


	buf[0] = FXOS8700CQ_WHOAMI;
		ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR_Magnetometer, buf, 1, HAL_MAX_DELAY);
		HAL_Delay(80);
		if ( ret == HAL_OK ) {

			ret = HAL_I2C_Master_Receive(&hi2c1, ADDR_Magnetometer, buf, 1, HAL_MAX_DELAY); /*empfange den Device Identifier*/
			HAL_Delay(80);
			if ( ret == HAL_OK ) {
				MagnetometerDeviceIdentifier = buf[0];
			}else{
				strcpy((char*)buf, "INIT ERR Read");
				MagnetometerDeviceIdentifier = 0xFF;
			}

		}else{
			strcpy((char*)buf, "INIT ERR Send");
			MagnetometerDeviceIdentifier = 0xFF;
		}

return MagnetometerDeviceIdentifier;
}


void MagnetometerWerteAuslesen (int16_t *x_axis_Mag, int16_t *y_axis_Mag, int16_t *z_axis_Mag){




	uint8_t buf[12]; /*ein Buffer-Array*/
	buf[0] = FXOS8700CQ_STATUS;
	HAL_StatusTypeDef ret;


	  /*hier ist die I2C-Übertragung*/

	  ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR_Magnetometer, buf, 1, HAL_MAX_DELAY); /*es wird ein byte gesendet, und zwar buf[0]*/
	  HAL_Delay(50);
	  if ( ret == HAL_OK ) {
		  ret = HAL_I2C_Master_Receive(&hi2c1, ADDR_Magnetometer, buf, 7, HAL_MAX_DELAY); /*empfange alle 6 Bytes für die Gyrowerte*/
		if ( ret == HAL_OK ) {
			*x_axis_Mag = 0;
			*x_axis_Mag = (buf[1] << 8) | buf[2]; /* buf[1] enthält x_MSB; buf[2] enthält x_LSB. Deshalb wird das MSB in die Variable eingesetzt, um 8 bits verschoben, und dann LSB mit logischem Oder verknüpft*/
			*y_axis_Mag = 0;
			*y_axis_Mag = (buf[3] << 8) | buf[4];
			*z_axis_Mag = 0;
			*z_axis_Mag = (buf[5] << 8) | buf[6];

		}else{
			strcpy((char*)buf, "Error Read");
		}
	  }else{
		  strcpy((char*)buf, "Error Write");
	  }
}

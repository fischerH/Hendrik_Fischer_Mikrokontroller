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
	buf[0] = CTRL_REG1;
	buf[1] = 0b00000000;

	//konfiguriere Control Register 1
		//versetze Sensor in Standby, um Control Register 1 ändern zu können ohne die Genauigkeit der Output Daten zu gefähreden -> Data Sheet S.45
	ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR_Gyro, buf, 2, HAL_MAX_DELAY);

		//konfiguriere Wert, der in CTRL_REG1 geschrieben werden soll
	buf[1] = 0b00001111; //Bit7: Unused		Bit6: Reset 0/1		Bit5: SelfTest 0/1		Bit4-2: Output Data Rate 011 für 100 Hz gewählt		Bit 1-0: Active Mode gewählt mit 11


	ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR_Gyro, buf, 2, HAL_MAX_DELAY);


	//überprüfe, ob Control Register 1 richtig konfiguriert wurde

	ret = HAL_I2C_Mem_Read(&hi2c1, ADDR_Gyro, CTRL_REG1, 1, buf, 1, 1000);
	//Kopiere Inhalt von buf[0] in buf[1]

	buf[1] = buf [0];

	//lese Device Identifier

 	ret = HAL_I2C_Mem_Read(&hi2c1, ADDR_Gyro, WHO_AM_I_Gyro_Reg, 1, buf, 1, 1000);



		if ( ret == HAL_OK && buf[0] == GyroDeviceID && buf[1] == 0b00001111) {
			//kein Hal-Fehler, GyroDeviceID ist korrekt, CTRL-Reg 1 hat richtige Werte
			return true;

		}else{
			strcpy((char*)buf, "INIT ERROR");
			return false;
		}

}



void gyroWerteAuslesen (int16_t *x_axis, int16_t *y_axis, int16_t *z_axis){
	HAL_StatusTypeDef ret;



	uint8_t buf[12]; /*ein Buffer-Array*/
	buf[0] = Gyro_Status_REG;



	  /*hier ist die I2C-Übertragung*/

	  ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR_Gyro, buf, 1, HAL_MAX_DELAY); /*es wird ein byte gesendet, und zwar buf[0]*/

	  if ( ret == HAL_OK ) {
		  //Burst-Read
		  ret = HAL_I2C_Master_Receive(&hi2c1, ADDR_Gyro, buf, 7, HAL_MAX_DELAY); /*empfange alle 6 Bytes für die Gyrowerte*/

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

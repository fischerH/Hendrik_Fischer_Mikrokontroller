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



	//versetze Sensor in Standby, um Control Register 1 ändern zu können ohne die Genauigkeit der Output Daten zu gefähreden -> Data Sheet S.45

	buf[0] = 0b00000000;
	ret = HAL_I2C_Mem_Write(&hi2c1, ADDR_Gyro, CTRL_REG1, 1, buf, 1, 1000);
	//konfiguriere Control Register 0

	buf[0] = CTRL_REG0;

	//Bit 7-6: LPF cutoff frequency wähle 01 für 16Hz bei ODR = 100 Hz		Bit 5: SPI Wire mode selection- unwichtig da I2C benutzt; wähle 0
	//Bit 4-3: High-pass filter cutoff frequency [00] irrelevant, da Bit 2 = 0		Bit 2: High-pass filter enable [0]
	//Bit 1-0: Full-scale range selection; wähle  10 für Range = +-500dps und Sensitivität = 15,625 mdps/LSB
	buf[1] = 0b01000010;

	ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR_Gyro, buf, 2, 1000);

	ret = HAL_I2C_Mem_Read(&hi2c1, ADDR_Gyro, CTRL_REG0, 1, buf, 1, 1000);

	if ( ret != HAL_OK || buf[0] != 0b01000010) {
		//prüfe ob I2C-Kommunikation geklappt hat und ob CTRL_Reg0 richtig konfiguriert ist
		strcpy((char*)buf, "INIT ERROR");
		return false;
	}
	//konfiguriere Control Register 1
	buf[0] = CTRL_REG1;
	buf[1] = 0b00001111; //Bit7: Unused		Bit6: Reset 0/1		Bit5: SelfTest 0/1		Bit4-2: Output Data Rate 011 für 100 Hz gewählt		Bit 1-0: Active Mode gewählt mit 11

		//aktiviere Sensor und konfiguriere CTRl_Reg1
	ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR_Gyro, buf, 2, 1000);


	//überprüfe, ob Control Register 1 richtig konfiguriert wurde

	ret = HAL_I2C_Mem_Read(&hi2c1, ADDR_Gyro, CTRL_REG1, 1, buf, 1, 1000);

	if ( ret != HAL_OK || buf[0] != 0b00001111) {
		//prüfe ob I2C-Kommunikation geklappt hat und ob CTRL_Reg1 richtig konfiguriert ist
		strcpy((char*)buf, "INIT ERROR");
		return false;
		}

	//lese Device Identifier

 	ret = HAL_I2C_Mem_Read(&hi2c1, ADDR_Gyro, WHO_AM_I_Gyro_Reg, 1, buf, 1, 1000);



	if ( ret != HAL_OK || buf[0] != GyroDeviceID) {
		//prüfe ob I2C-Kommunikation geklappt hat und ob die Device ID korrekt ist
		strcpy((char*)buf, "INIT ERROR");
		return false;
		}
return true;
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
			*x_axis = ((buf[1] << 8) | buf[2]);//*15.625; /* buf[1] enthält x_MSB; buf[2] enthält x_LSB. Deshalb wird das MSB in die Variable eingesetzt, um 8 bits verschoben, und dann LSB mit logischem Oder verknüpft*/
			*y_axis = 0;
			*y_axis = ((buf[3] << 8) | buf[4]);// * 15.625; //Nur Rohdaten. Jeweils multipliziert mit Sensitivity Value, um milli-degrees per second zu erhalten, siehe datasheet S. 27 bzw. Tabell 35 (+-500dps gewählt)
			*z_axis = 0;
			*z_axis = ((buf[5] << 8) | buf[6]);//*15.625;

		}else{
			strcpy((char*)buf, "Error Read");
		}
	  }else{
		  strcpy((char*)buf, "Error Write");
	  }
}

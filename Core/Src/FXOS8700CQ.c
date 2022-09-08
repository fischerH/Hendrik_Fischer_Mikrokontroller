/*
 * Magnetometer.c
 *
 *  Created on: 20.07.2022
 *      Author: Hendr
 */

#include <FXOS8700CQ.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>


bool InitialisiereFXOS8700CQ(){
	HAL_StatusTypeDef ret;
	uint8_t buf[15];


	//Versetze Sensor zur Konfiguration in Standby

	buf[0] = FXOS8700CQ_CTRL_REG1;
	buf[1] = 0b00000000; //letztes Bit gibt Standby-Modus an
	ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR_Magnetometer, buf, 2, 1000);

	//konfiguriere M_CTRL_REG1
	buf[0] = FXOS8700CQ_M_CTRL_REG1;

			//Bit 7: Auto Calibration [On]/Off	Bit 6: One-shot magnetic reset On/[Off]
			//Bit 5: One-shot triggered Magnetic measurement mode On/[Off]
			//Bit 4-2: Oversample ratio (OSR) (Datasheet S.99) wähle 011 für OSR = 16 bei 6,25 Hz ODR
			//Bit 1-0:	11 gewählt für Hybrid Mode
	buf[1] = 0b10001111;

	ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR_Magnetometer, buf, 2, 1000);

		//prüfe, ob M_CTRL_REG1 korrekt konfiguriert ist

	ret = HAL_I2C_Mem_Read(&hi2c1, ADDR_Magnetometer, FXOS8700CQ_M_CTRL_REG1, 1, buf, 1, 1000);

			if (ret != HAL_OK || buf[0] != 0b10001111){
				//prüfe ob I2C-Kommunikation geklappt hat bzw. ob die M_CTRL-REG1-Werte richtig sind
				strcpy((char*)buf, "INIT ERROR");
				return false;
			}
	//konfiguriere M_CTRL_REG2

	buf[0] = FXOS8700CQ_M_CTRL_REG2;

			//Bit 7-6: nicht belegt; unwichtig
			//Bit 5: hyb_autoinc_mod = 1, um Mag- und Acc- Daten in einem Burst read auszulesen
			//Bit 4-2: Magnetic measurement max/min-Konfiguration, für Anwendung unwichtig, wähle deshalb Standardwert 000
			//Bit 1-0: Magnetic sensor reset (degaussing) Frequenz; wähle Standardwert 00
	buf[1] = 0b00100000;

	ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR_Magnetometer, buf, 2, 1000);

	ret = HAL_I2C_Mem_Read(&hi2c1, ADDR_Magnetometer, FXOS8700CQ_M_CTRL_REG2, 1, buf, 1, 1000);

			if (ret != HAL_OK || buf[0] != 0b00100000){
				//prüfe ob I2C-Kommunikation geklappt hat bzw. ob die M_CTRL-REG2-Werte richtig sind
				strcpy((char*)buf, "INIT ERROR");
				return false;
			}
	//aktiviere Sensor und konfiguriere Control Register 1
		//Bit 7-6: auto-wake sample frequency; irrelevant (wähle 10)	Bit 5-3: Output data rate selection; wähle 101 für 12,5Hz mag only mode
		//bzw. 6,25 Hz hybrid mode	Bit 2: Inoise; wähle 1 für Reduced Noise mode	Bit 1: Fast Read Mode; wähle 0 für Normal Mode	Bit 0: wähle 1
		//um Sensor aus Standby zu holen und zu aktivieren
	buf[0] = FXOS8700CQ_CTRL_REG1;
	buf[1] = 0b10101101;
	ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR_Magnetometer, buf, 2, HAL_MAX_DELAY);
		//prüfe, ob CTRL_REG1 richtig konfiguriert wurde
	ret = HAL_I2C_Mem_Read(&hi2c1, ADDR_Magnetometer, FXOS8700CQ_CTRL_REG1, 1, buf, 1, 1000);

		if (ret != HAL_OK || buf[0] != 0b10101101){
			//prüfe ob I2C-Kommunikation geklappt hat bzw. ob die CTRL-REG1-Werte richtig sind
			strcpy((char*)buf, "INIT ERROR");
			return false;
		}


	//checke den Device Identifier

	ret = HAL_I2C_Mem_Read(&hi2c1, ADDR_Magnetometer, FXOS8700CQ_WHOAMI, 1, buf, 1, 1000);

		if ( ret != HAL_OK || buf[0] != FXOS8700CQ_WHOAMI_VAL) {
			//prüfe ob I2C-Kommunikation geklappt hat bzw. ob die Device-ID richtig ist
			strcpy((char*)buf, "INIT ERROR");
			return false;
	}
return true;
}




void FXOS8700CQWerteAuslesen (int16_t *x_axis_Mag, int16_t *y_axis_Mag, int16_t *z_axis_Mag, int16_t *x_axis_Acc, int16_t *y_axis_Acc, int16_t *z_axis_Acc){


	uint8_t buf[15]; /*ein Buffer-Array*/
	buf[0] = FXOS8700CQ_STATUS;
	HAL_StatusTypeDef ret;


	  //I2C Burst Read. Startet mit dem Status Register. Autoinkrement zu 0x06, dann zu 0x33 (siehe Konfiguration M_CTRL_REG2)

	  ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR_Magnetometer, buf, 1, HAL_MAX_DELAY); /*es wird ein byte gesendet, und zwar buf[0]*/

	  if ( ret == HAL_OK ) {
		  ret = HAL_I2C_Master_Receive(&hi2c1, ADDR_Magnetometer, buf, 13, HAL_MAX_DELAY); /*empfange alle 6 Bytes für die Gyrowerte*/
		if ( ret == HAL_OK ) {
			*x_axis_Acc = 0;
			*x_axis_Acc = (int16_t)(((buf[1] << 8) | buf[2])) >> 2; // buf[1] enthält x_MSB; buf[2] enthält x_LSB. Deshalb wird das MSB in die Variable eingesetzt, um 8 bits verschoben, und dann LSB mit logischem Oder verknüpft
			*y_axis_Acc = 0;
			*y_axis_Acc = (int16_t)(((buf[3] << 8) | buf[4])) >> 2; //Accel-Data ist 14bit, deshalb anschleißend BitShift um 2
			*z_axis_Acc = 0;
			*z_axis_Acc = (int16_t)(((buf[5] << 8) | buf[6])) >> 2;
			*x_axis_Mag = 0;
			*x_axis_Mag = (buf[7] << 8) | buf[8]; // buf[7] enthält x_MSB; buf[2] enthält x_LSB. Deshalb wird das MSB in die Variable eingesetzt, um 8 bits verschoben, und dann LSB mit logischem Oder verknüpft
			*y_axis_Mag = 0;
			*y_axis_Mag = (buf[9] << 8) | buf[10];
			*z_axis_Mag = 0;
			*z_axis_Mag = (buf[11] << 8) | buf[12];

		}else{
			strcpy((char*)buf, "Error Read");
		}
	  }else{
		  strcpy((char*)buf, "Error Write");
	  }
}

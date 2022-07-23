/*
 * gyro.h
 *
 *  Created on: 15.07.2022
 *      Author: Hendr
 */

#ifndef INC_GYRO_H_
#define INC_GYRO_H_



#ifndef Gyro_Treiber_H


#define Gyro_Treiber_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include <stdbool.h>



#define ADDR_Gyro (0x21 << 1) //Initialisiere Adresse von Gyro Sensor. Wert aus Datenblatt. Shift ein Bit da I2C 7 Bit Adresse verwendet und letztes Bit 0 sein muss.

#define Gyro_Status_REG (0x0)
#define CTRL_REG1 (0x13)
#define CTRL_REG2 (0x14)
#define CTRL_REG3 (0x15)
#define WHO_AM_I_Gyro_Reg (0x0C)
#define GyroDeviceID (0xD7)

extern 	I2C_HandleTypeDef hi2c1;

void gyroWerteAuslesen (int16_t *x_axis, int16_t *y_axis, int16_t *z_axis);

bool InitialisiereGyro();

#ifdef __cplusplus
}
#endif

#endif

#endif /* INC_GYRO_H_ */

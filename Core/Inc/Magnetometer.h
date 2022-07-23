/*
 * Magnetometer.h
 *
 *  Created on: 16.07.2022
 *      Author: Hendr
 */

#ifndef INC_MAGNETOMETER_H_
#define INC_MAGNETOMETER_H_



#ifndef Magnetometer_Treiber_H


#define GMagnetometer_Treiber_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include <stdbool.h>

#define ADDR_Magnetometer  0x1F << 1 //Initialisiere Adresse von Magnetometer Sensor. Wert aus Datenblatt. Shift ein Bit da I2C 7 Bit Adresse verwendet und letztes Bit 0 sein muss.


// FXOS8700CQ internal register addresses; kopiert aus Datasheet Seite 25
#define FXOS8700CQ_STATUS 0x00
#define FXOS8700CQ_WHOAMI 0x0D
#define FXOS8700CQ_XYZ_DATA_CFG 0x0E
#define FXOS8700CQ_CTRL_REG1 0x2A
#define FXOS8700CQ_M_CTRL_REG1 0x5B
#define FXOS8700CQ_M_CTRL_REG2 0x5C
#define FXOS8700CQ_WHOAMI_VAL 0xC7

extern 	I2C_HandleTypeDef hi2c1;


bool InitialisiereMagnetometer();
void MagnetometerWerteAuslesen (int16_t *x_axis_Mag, int16_t *y_axis_Mag, int16_t *z_axis_Mag);

#ifdef __cplusplus
}
#endif

#endif
#endif /* INC_MAGNETOMETER_H_ */

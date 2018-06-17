/*

Robot Main Brain  --  runs on 1284P and handles onboard control of my robot
     Copyright (C) 2017  David C.

     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.

     */

#ifndef DEFINES_H_
#define DEFINES_H_

#include <RobotSharedDefines.h>


/*
 *
 *                    +---\/---+
 right Mot (D 0) PB0 1|        |40 PA0 (AI 0 / D24)     battery
 left mot  (D 1) PB1 2|        |39 PA1 (AI 1 / D25)
      INT2 (D 2) PB2 3|        |38 PA2 (AI 2 / D26)
right enWM (D 3) PB3 4|        |37 PA3 (AI 3 / D27)
lt enWM/SS (D 4) PB4 5|        |36 PA4 (AI 4 / D28)    cam enable
      MOSI (D 5) PB5 6|        |35 PA5 (AI 5 / D29)    arm power enable
  PWM/MISO (D 6) PB6 7|        |34 PA6 (AI 6 / D30)    com power enable
   PWM/SCK (D 7) PB7 8|        |33 PA7 (AI 7 / D31)    heartbeat
                 RST 9|        |32 AREF
                VCC 10|        |31 GND
                GND 11|        |30 AVCC
              XTAL2 12|        |29 PC7 (D 23)          Motor Encoder
              XTAL1 13|        |28 PC6 (D 22)          Motor Encoder
      RX0 (D 8) PD0 14|        |27 PC5 (D 21) TDI      Motor Encoder
      TX0 (D 9) PD1 15|        |26 PC4 (D 20) TDO      Motor Encoder
RX1/INT0 (D 10) PD2 16|        |25 PC3 (D 19) TMS
TX1/INT1 (D 11) PD3 17|        |24 PC2 (D 18) TCK
gimbPWM (D 12) PD4 18|        |23 PC1 (D 17) SDA
gimb PWM (D 13) PD5 19|        |22 PC0 (D 16) SCL
ping PWM (D 14) PD6 20|        |21 PD7 (D 15) PWM   headlight
                      +--------+
 *
 *
 *
 */

#define RIGHT_MOTOR_DIRECTION_PIN 0
#define LEFT_MOTOR_DIRECTION_PIN 1


#define RIGHT_MOTOR_ENABLE_PIN 3
#define LEFT_MOTOR_ENABLE_PIN 4


#define GIMBAL_PAN_SERVO_PIN 12
#define GIMBAL_TILT_SERVO_PIN 13

#define PING_SENSOR_PIN 14

#define HEADLIGHT_PIN 15

#define CAM_ENABLE 28
#define ARM_ENABLE 29
#define COM_POWER_ENABLE 30

#define HEARTBEAT_PIN 31

#define BATTERY_PIN A0




#define NUMBER_BATTERY_READINGS_TO_AVERAGE 30




#endif /* DEFINES_H_ */

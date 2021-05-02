/*

Robot Main Brain  --  runs on 1284P and handles onboard control of my robot
     Copyright (C) 2020  David C.

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
 *                           ATMEGA1284P
 *
 *
 *                                   +---\/---+
                xp0 Reset (D 0) PB0 1|        |40 PA0 (AI 0 / D24)     battery
                xp0 CS    (D 1) PB1 2|        |39 PA1 (AI 1 / D25)     M1FB
                xp0 INT   (D 2) PB2 3|        |38 PA2 (AI 2 / D26)     M2FB
                 M1D2 ~   (D 3) PB3 4|        |37 PA3 (AI 3 / D27)
                 M2D2 ~   (D 4) PB4 5|        |36 PA4 (AI 4 / D28)
                     MOSI (D 5) PB5 6|        |35 PA5 (AI 5 / D29)
                     MISO (D 6) PB6 7|        |34 PA6 (AI 6 / D30)    Gimbal Tilt
                      SCK (D 7) PB7 8|        |33 PA7 (AI 7 / D31)    Gimbal Pan
                                RST 9|        |32 AREF
                               VCC 10|        |31 GND
                               GND 11|        |30 AVCC
                             XTAL2 12|        |29 PC7 (D 23)          Motor Encoder
                             XTAL1 13|        |28 PC6 (D 22)          Motor Encoder
                     RX0 (D 8) PD0 14|        |27 PC5 (D 21) TDI      Motor Encoder
                     TX0 (D 9) PD1 15|        |26 PC4 (D 20) TDO      Motor Encoder
               RX1/INT0 (D 10) PD2 16|        |25 PC3 (D 19) TMS      M1IN2
               TX1/INT1 (D 11) PD3 17|        |24 PC2 (D 18) TCK      M1IN1
              HeartBeat (D 12) PD4 18|        |23 PC1 (D 17) SDA      M2IN2
               trig PWM (D 13) PD5 19|        |22 PC0 (D 16) SCL      M2IN1
          ICP1 echo PWM (D 14) PD6 20|        |21 PD7 (D 15) PWM      adc CS
                                     +--------+
 *
 *
 *
 *             MCP23S08 expander xp0  (Main Brain Board)
 *
 *                                   +---\/---+
 *                            SCK   1|        |18 VDD
 *                             SI   2|        |17 GP7 (107)   Motor Controller Enable
 *                             SO   3|        |16 GP6 (106)   Arm Enable
 *                             A1   4|        |15 GP5 (105)   Sonar Enable
 *                             A0   5|        |14 GP4 (104)   M1SF Status Flag
 *                          _RESET  6|        |13 GP3 (103)   M2SF Status Flag
 *                             _CS  7|        |12 GP2 (102)
 *                       INTERRUPT  8|        |11 GP1 (101)
 *                             VSS  9|        |10 GP0 (100)
 *                                   +--------+
 *
 *
 *             MCP23S08 expander xp1  (Power Board)
 *
 *                                   +---\/---+
 *                            SCK   1|        |18 VDD
 *                             SI   2|        |17 GP7 (115)   Motor Power Enable
 *                             SO   3|        |16 GP6 (114)
 *                             A1   4|        |15 GP5 (113)
 *                             A0   5|        |14 GP4 (112)   12V Power Rail Enable
 *                          _RESET  6|        |13 GP3 (111)   Headlight On
 *                             _CS  7|        |12 GP2 (110)   Camera On
 *                       INTERRUPT  8|        |11 GP1 (109)   Aux Power Enable
 *                             VSS  9|        |10 GP0 (108)   Radio Enable
 *                                   +--------+
 *
 *
 *
 *             MCP3008 ADC adc  (Power Board)
 *
 *                                   +---\/---+
 *                  Battery   CH0   1|        |16 VDD
 *               12V System   CH1   2|        |15 Vref
 *                            CH2   3|        |14 Analog Ground
 *                            CH3   4|        |13 CLK
 *                Aux power   CH4   5|        |12 Data_OUT
 *                 Grounded   CH5   6|        |11 Data_IN
 *                  Main 5V   CH6   7|        |10 _CS
 *              Radio Power   CH7   8|        |9  Digital Ground
 *                                   +--------+
 *

 *
 *
 *
 */

#define XPANDER_RESET_PIN 0
#define XPANDER_CS_PIN 1
#define MB_XPANDER_HW_ADDY 0
#define POWER_XPANDER_HW_ADDY 3

#define POWER_ADC_CS_PIN 15

#define LEFT_MOTOR_SF_PIN 3
#define RIGHT_MOTOR_SF_PIN 4


#define LEFT_MOTOR_DIRECTION_PIN_1 16
#define LEFT_MOTOR_DIRECTION_PIN_2 17
#define RIGHT_MOTOR_DIRECTION_PIN_1 18
#define RIGHT_MOTOR_DIRECTION_PIN_2 19


#define LEFT_MOTOR_ENABLE_PIN 4
#define RIGHT_MOTOR_ENABLE_PIN 3

#define LEFT_MOTOR_FEEDBACK_PIN A2
#define RIGHT_MOTOR_FEEDBACK_PIN A1

#define GIMBAL_PAN_SERVO_PIN 30
#define GIMBAL_TILT_SERVO_PIN 31
//
//#define PING_SENSOR_PIN 14




#define SONAR_ENABLE_PIN 105
#define ARM_ENABLE_PIN 106
#define MOTOR_CONTROLLER_ENABLE_PIN 107
#define COM_POWER_ENABLE_PIN 108
#define AUX_POWER_ENABLE_PIN 109
#define CAM_ENABLE_PIN 110
#define HEADLIGHT_PIN 111
#define V12_ENABLE 112

#define MOTOR_POWER_ENABLE_PIN 115


#define HEARTBEAT_PIN 12

#define BATTERY_PIN A0

enum VoltageEnum {
	VOLT_ENUM_BATTERY,
	VOLT_ENUM_V12,
	VOLT_ENUM_AUX,
	VOLT_ENUM_MAIN5,
	VOLT_ENUM_RADIO,
	VOLT_ENUM_MOTOR
};

#define BATTERY_ADC_PIN 0
#define V12_ADC_PIN 1
#define AUX_ADC_PIN 4
#define MAIN5_ADC_PIN 6
#define RADIO_ADC_PIN 7

#define BATTERY_ADC_CAL_FACTOR 24.37
#define V12_ADC_CAL_FACTOR 17.45
#define AUX_ADC_CAL_FACTOR 9.25
#define MAIN5_ADC_CAL_FACTOR 9.41
#define RADIO_ADC_CAL_FACTOR 9.54


#define NUMBER_BATTERY_READINGS_TO_AVERAGE 30
#define VOLTAGE_READING_INTERVAL 1000
#define VOLTAGE_REPORTING_INTERVAL 10000






/*              OLD BOARD STUFF
 *
 *                                   +---\/---+
                right Mot (D 0) PB0 1|        |40 PA0 (AI 0 / D24)     battery
                left mot  (D 1) PB1 2|        |39 PA1 (AI 1 / D25)
                     INT2 (D 2) PB2 3|        |38 PA2 (AI 2 / D26)
               right enWM (D 3) PB3 4|        |37 PA3 (AI 3 / D27)
               lt enWM/SS (D 4) PB4 5|        |36 PA4 (AI 4 / D28)    cam power enable
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
               RX1/INT0 (D 10) PD2 16|        |25 PC3 (D 19) TMS      Gimbal Pan
               TX1/INT1 (D 11) PD3 17|        |24 PC2 (D 18) TCK      Gimbal Tilt
                    PWM (D 12) PD4 18|        |23 PC1 (D 17) SDA
               trig PWM (D 13) PD5 19|        |22 PC0 (D 16) SCL
          ICP1 echo PWM (D 14) PD6 20|        |21 PD7 (D 15) PWM   headlight
                                     +--------+
 *
 *
 *
 */

/*

#define RIGHT_MOTOR_DIRECTION_PIN 0
#define LEFT_MOTOR_DIRECTION_PIN 1


#define RIGHT_MOTOR_ENABLE_PIN 3
#define LEFT_MOTOR_ENABLE_PIN 4


//#define GIMBAL_PAN_SERVO_PIN 12
//#define GIMBAL_TILT_SERVO_PIN 13
//
//#define PING_SENSOR_PIN 14

#define HEADLIGHT_PIN 15

#define CAM_ENABLE 28
#define ARM_ENABLE 29
#define COM_POWER_ENABLE 30

#define HEARTBEAT_PIN 31

#define BATTERY_PIN A0




#define NUMBER_BATTERY_READINGS_TO_AVERAGE 30

*/


#endif /* DEFINES_H_ */

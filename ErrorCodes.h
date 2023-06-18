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


#ifndef ERRORCODES_H_
#define ERRORCODES_H_

#include "Arduino.h"

void sendError(uint8_t aErrorCode);

#define ECODE_BAD_ERROR_CODE 127

#define ECODE_SELF_TEST_FAIL 1
#define ECODE_BAD_RAW_INPUT 10


#define ECODE_XPANDER_SPI_FAIL 11
#define ECODE_POWERXPANDER_SPI_FAIL 12
#define ECODE_POWERADC_SPI_FAIL 13


#define ECODE_FAKE_CODE 19

#define ECODE_ARM_BOOT_TIMEOUT 101
#define ECODE_ARM_BAD_N 102




#endif /* ERRORCODES_H_ */

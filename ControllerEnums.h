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

#ifndef CONTROLLERENUMS_H_
#define CONTROLLERENUMS_H_


///** Buttons on the controllers */
//const uint16_t buttonMasks[] PROGMEM = {
//        0x0100, // UP
//        0x0800, // RIGHT
//        0x0200, // DOWN
//        0x0400, // LEFT
//
//        0x2000, // BACK
//        0x1000, // START
//        0x4000, // L3
//        0x8000, // R3
//
//        0, 0, // Skip L2 and R2 as these are analog buttons
//        0x0001, // L1
//        0x0002, // R1
//
//        0x0020, // B
//        0x0010, // A
//        0x0040, // X
//        0x0080, // Y
//
//        0x0004, // XBOX
//        0x0008, // SYNC
//};



enum ButtonMaskEnum {
		UP = 0x0100,
		RIGHT = 0x0800,
		DOWN = 0x0200,
		LEFT = 0x0400,
		BACK = 0x2000,
		START = 0x1000,
		L3 = 0x4000,
		R3 = 0x8000,
		L2 = 0xFF00,
		R2 = 0x00FF,
		L1 = 0x0001,
		R1 = 0x0002,

		B = 0x0020,
		A = 0x0010,
		X = 0x0040,
		Y = 0x0080,

		XBOX = 0x0004,
		SYNC = 0x0008,
};


//enum ButtonEnum {
//	UP = 0,
//	RIGHT = 1,
//	DOWN = 2,
//	LEFT = 3,
//	BACK = 4,
//	START = 5,
//	L3 = 6,
//	R3 = 7,
//	L2 = 8,
//	R2 = 9,
//	L1 = 10,
//	R1 = 11,
//
//	B = 12,
//	A = 13,
//	X = 14,
//	Y = 15,
//
//	XBOX = 16,
//	SYNC = 17,
//};

/** Joysticks on the PS3 and Xbox controllers. */
enum HatEnum {
        LeftHatX = 0,
        LeftHatY = 1,
        RightHatX = 2,
        RightHatY = 3,
};





#endif /* CONTROLLERENUMS_H_ */

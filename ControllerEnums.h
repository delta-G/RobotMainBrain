/*
 * ControllerEnums.h
 *
 *  Created on: Jun 16, 2017
 *      Author: david
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
		L2 = 0,
		R2 = 0,
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
//enum AnalogHatEnum {
//        /** Left joystick x-axis */
//        LeftHatX = 0,
//        /** Left joystick y-axis */
//        LeftHatY = 1,
//        /** Right joystick x-axis */
//        RightHatX = 2,
//        /** Right joystick y-axis */
//        RightHatY = 3,
//};





#endif /* CONTROLLERENUMS_H_ */

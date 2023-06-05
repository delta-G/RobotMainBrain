#PROJECT_ROOT = $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
#
#OBJS = 328HelloWorld.o
#
#ifeq ($(BUILD_MODE),debug)
#	CFLAGS += -g
#else ifeq ($(BUILD_MODE),run)
#	CFLAGS += -O2
#else ifeq ($(BUILD_MODE),linuxtools)
#	CFLAGS += -g -pg -fprofile-arcs -ftest-coverage
#	LDFLAGS += -pg -fprofile-arcs -ftest-coverage
#	EXTRA_CLEAN += 328HelloWorld.gcda 328HelloWorld.gcno $(PROJECT_ROOT)gmon.out
#	EXTRA_CMDS = rm -rf 328HelloWorld.gcda
#else
#    $(error Build mode $(BUILD_MODE) not supported by this Makefile)
#endif

ARDUINO_DIR = /home/david/.arduino15
ALTERNATE_CORE_PATH = /home/david/.arduino15/packages/MightyCore/hardware/avr/2.2.1
ARDMK_DIR = /usr/share/arduino
AVR_TOOLS_DIR = /usr/bin

USER_LIB_PATH = /home/david/Arduino/libraries
ARDUINO_PLATFORM_LIB_PATH = /home/david/.arduino15/packages/MightyCore/hardware/avr/2.2.1/libraries
AVRDUDE_CONF = /etc/avrdude.conf
AVRDUDE_ARD_PROGRAMMER = arduino
AVRDUDE_ARD_BAUDRATE = 115200

TARGET = RobotMainBrain.cpp

#BOARD_TAG    = uno  # for mega use mega2560
# --- mighty 1284p
BOARD_TAG         = 1284
MCU	=	atmega1284p
VARIANT = standard

F_CPU = 16000000

BOARDS_TXT        = /home/david/.arduino15/packages/MightyCore/hardware/avr/2.2.1/boards.txt
BOOTLOADER_PARENT = /home/david/.arduino15/packages/MightyCore/hardware/avr/2.2.1/bootloaders
BOOTLOADER_PATH   = optiboot
BOOTLOADER_FILE   = optiboot_atmega1284p.hex
ISP_PROG     	   = usbasp
AVRDUDE_OPTS 	   = -v
ARDUINO_PORT = /dev/ttyUSB0  # change this to the port used by your board

ARDUINO_LIBS = RobotSharedDefines
ARDUINO_LIBS += CommandParser
ARDUINO_LIBS += XboxHandler
ARDUINO_LIBS += PID_v1
ARDUINO_LIBS += PingTimer
ARDUINO_LIBS += Gimbal
ARDUINO_LIBS += Joint
ARDUINO_LIBS += Servo
ARDUINO_LIBS += EepromFuncs
ARDUINO_LIBS += EEPROM
ARDUINO_LIBS += ServoCalibration
ARDUINO_LIBS += MCP23S08
ARDUINO_LIBS += MCP3008
ARDUINO_LIBS += StreamParser

AVR_TOOLS_PATH = /usr/bin

CPPFLAGS += -DARDUINO_ARCH_AVR
CPPFLAGS += -D__AVR_ATmega1284P__

PRE_BUILD_HOOK = ../preCompile.sh

include /usr/share/arduino/Arduino.mk



#all:	328HelloWorld
#
#328HelloWorld:	$(OBJS)
#	$(CXX) $(LDFLAGS) -o $@ $^
#	$(EXTRA_CMDS)
#
#%.o:	$(PROJECT_ROOT)%.cpp
#	$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(CPPFLAGS) -o $@ $<
#
#%.o:	$(PROJECT_ROOT)%.c
#	$(CC) -c $(CFLAGS) $(CPPFLAGS) -o $@ $<
#
#clean:
#	rm -fr 328HelloWorld $(OBJS) $(EXTRA_CLEAN)

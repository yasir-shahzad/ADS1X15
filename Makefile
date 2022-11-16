CXX = g++
C=gcc -c
STANDARD=-std=c++17
WNO_FLAG=-Wno-psabi

#DEBUGGING FLAGS
PTHREAD=-lpthread
CXXFLAGS=-Wall -g3
LIB_CURL=-lcurl
LIB_FS=-lstdc++fs
FCONCEPTS=-fconcepts

#COMMON PARAMS
COMMON_FLAGS=$(CXX) $(STANDARD) $(WNO_FLAG) $(FCONCEPTS)
DEBUG_FLAG=BUILD_RELEASE


.PHONY: adx

all: adx
adx: main.cpp
	$(COMMON_FLAGS)  main.cpp Adafruit_ADS1x15.cpp -fconcepts -li2c -o adx
#	g++ -std=c++17 main.cpp   tb6600.cpp ../../Libraries/PCA9685/PCA9685.cpp #../../Libraries/Gpio_Expander/src/MCP2317.cpp -o motor



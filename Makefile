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


.PHONY: adx1x15

all: adx1x15
adx1x15: examples/singleended.cpp
	$(COMMON_FLAGS)  examples/singleended.cpp src/ADS1x15.cpp -fconcepts -li2c -o bin/adx1x15




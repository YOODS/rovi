# Makefile for Sentech grabber

.PHONY: all clean

TARGET1:=grabber

COMPILEFLAGS:=-Wall -Wextra --pedantic -Wno-unknown-pragmas -Wno-switch \
		-Wno-unused-but-set-variable -Wno-pointer-arith -Wno-unused-parameter \
		-std=c++11

INCFLAGS:=-I${STAPI_ROOT_PATH}/include/GenICam \
		-I${STAPI_ROOT_PATH}/include/StApi

LIBFLAGS:=-L${STAPI_ROOT_PATH}/lib \
		-L${STAPI_ROOT_PATH}/lib/GenICam \
		-lGCBase \
		-lGenApi \
		-lpthread

all: $(TARGET1)

$(TARGET1): $(TARGET1).cpp cparser.c
	$(CXX) $^ -o $@ $(COMPILEFLAGS) $(INCFLAGS) $(LIBFLAGS) -lStApi_TL

clean:
	$(RM) $(TARGET1)

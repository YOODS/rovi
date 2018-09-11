SL       = /
include ./macros.mak
#SRCS = yod_A02.cpp
SRCS = main.c noise_reduct.c voxel.c plyio.c point_mesh.c mycommons.c
#OBJS = $(SRCS:.cpp=.o)
OBJS = main.o noise_reduct.o voxel.o plyio.o point_mesh.o mycommons.o
#TARGET = libyodpy.so
TARGET = yodpy2
CXX = g++
CXXFLAGS = -O3 -Wall -fPIC -std=c++14 -fPIC `python3 -m pybind11 --includes` 
LDFLAGS = -shared
$LIBS = 

AR_EXT         = so
HINCLUDES      = -I$(HINC_DIR) -I$(HCPPINC_DIR) -I/usr/include/eigen3
HLIBRARIES     = -L$(HLIB_DIR) -lhalconcpp$(HSUFFIX) -lhalcon$(HSUFFIX) -L/usr/lib

ifeq ($(PLATFORM),x64)
        LIBRARIES       = -L/usr/X11R6/lib64 -lXext -lX11 -lpthread -lm -ldl
        PLATFORM_CFLAGS = $(HINCLUDES)

else
#   x86sse2 linux architecture
        LIBRARIES       = -L/usr/X11R6/lib -lXext -lX11 -lpthread -lm -ldl
        PLATFORM_CFLAGS = -march=pentium4 -msse2 $(HINCLUDES)
endif

all: $(OBJS)
	$(CXX) $(LDFLAGS) $(OBJS) $(LIBS) $(HLIBRARIES) $(HINCLUDES) -o $(TARGET)`python3-config --extension-suffix`
	@echo done.

$(OBJS): $(SRCS)
	$(CXX) $(CXXFLAGS) $(HINCLUDES) $(SRCS) -c

clean:
	rm -rf *.o core.* *# *~ $(TARGET)`python3-config --extension-suffix`


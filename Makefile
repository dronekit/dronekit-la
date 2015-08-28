# To cross compile:
#
# Set up as usual for bitbake:
# $ . setup-environment build
#
# In the build directory:
# $ bitbake meta-ide-support
# $ . tmp/environment-setup-cortexa9hf-vfp-neon-poky-linux-gnueabi
#
# Now a make in this directory should work.

GIT_VERSION := $(shell git describe --abbrev=4 --dirty --always --tags)

VPATH = ./util ./ini ./ini/cpp

INCS = -I./util -I./ini -I./ini/cpp

STD=-std=c++11
CFLAGS += -Wall $(INCS) -DGIT_VERSION=\"$(GIT_VERSION)\"
CXXFLAGS += -Wall $(INCS) $(STD) -g -DGIT_VERSION=\"$(GIT_VERSION)\"

DLIBS += -ljsoncpp

# SRCS_CPP = dataflash_logger.cpp
SRCS_CPP += INIReader.cpp
SRCS_CPP += analyzer_util.cpp
SRCS_CPP += mavlink_message_handler.cpp
SRCS_CPP += mavlink_reader.cpp
SRCS_CPP += analyze.cpp
SRCS_CPP += analyzer.cpp
SRCS_CPP += heart.cpp
SRCS_CPP += analyzer_compass_offsets.cpp
SRCS_CPP += analyzer_ever_armed.cpp
SRCS_CPP += analyzer_ever_flew.cpp
SRCS_CPP += analyzer_good_ekf.cpp
SRCS_CPP += analyzer_battery.cpp
SRCS_CPP += analyzer_brownout.cpp
SRCS_CPP += analyzer_notcrashed.cpp
SRCS_CPP += analyzer_attitude_control.cpp
SRCS_CPP += analyzervehicle_copter.cpp
SRCS_CPP += analyzervehicle.cpp
SRCS_CPP += la-log.cpp
SRCS_CPP += common_tool.cpp
SRCS_C = util.c ini.c

OBJS = $(SRCS_CPP:.cpp=.o) $(SRCS_C:.c=.o)

DATAFLASH_LOGGER = dataflash_logger

LOG_ANALYZER = loganalyzer

all: $(DATAFLASH_LOGGER) $(LOG_ANALYZER)

$(DATAFLASH_LOGGER): $(OBJS) dataflash_logger.cpp
	$(LINK.cpp) -o $(DATAFLASH_LOGGER) dataflash_logger.cpp $(OBJS) $(LIBS) $(DLIBS)

$(LOG_ANALYZER): $(OBJS) loganalyzer.cpp
	$(LINK.cpp) -o $(LOG_ANALYZER) loganalyzer.cpp $(OBJS) $(LIBS) $(DLIBS)

clean:
	$(RM) *.o *~ $(DATAFLASH_LOGGER) $(LOG_ANALYZER)

test: clean all
	cd test; ./test.sh

.PHONY: clean

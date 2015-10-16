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
INCS += -I.  # for <DataFlash/DataFlash.h> in MsgHandler

STD=-std=c++11
#STATIC=-static
WARNFLAGS= -Wall -Werror -Wextra -Wunused -Wlogical-op -Wredundant-decls -D_FORTIFY_SOURCE=2 -Wfloat-equal -fstack-protector
CFLAGS += $(INCS) -DGIT_VERSION=\"$(GIT_VERSION)\" $(WARNFLAGS)
CXXFLAGS += $(INCS) $(STD) -g -DGIT_VERSION=\"$(GIT_VERSION)\" $(STATIC) $(WARNFLAGS)

DLIBS += -ljsoncpp

SRCS_CPP += INIReader.cpp
SRCS_CPP += analyzer_util.cpp
SRCS_CPP += mavlink_message_handler.cpp
SRCS_CPP += message_handler.cpp
SRCS_CPP += MsgHandler.cpp
SRCS_CPP += format_reader.cpp
SRCS_CPP += dataflash_reader.cpp
SRCS_CPP += dataflash_textdump_reader.cpp
SRCS_CPP += mavlink_reader.cpp
SRCS_CPP += mavlink_writer.cpp
SRCS_CPP += analyze.cpp
SRCS_CPP += analyzer.cpp
SRCS_CPP += heart.cpp
SRCS_CPP += analyzer/analyzer_any_parameters_seen.cpp
SRCS_CPP += analyzer/analyzer_arming_checks.cpp
SRCS_CPP += analyzer/analyzer_attitude_control.cpp
SRCS_CPP += analyzer/analyzer_battery.cpp
SRCS_CPP += analyzer/analyzer_brownout.cpp
SRCS_CPP += analyzer/analyzer_compass_offsets.cpp
SRCS_CPP += analyzer/analyzer_compass_vector_length.cpp
SRCS_CPP += analyzer/analyzer_ever_armed.cpp
SRCS_CPP += analyzer/analyzer_ever_flew.cpp
SRCS_CPP += analyzer/analyzer_good_ekf.cpp
SRCS_CPP += analyzer/analyzer_gps_fix.cpp
SRCS_CPP += analyzer/analyzer_notcrashed.cpp
SRCS_CPP += analyzer/analyzer_sensor_health.cpp
SRCS_CPP += analyzer/analyzer_estimate_divergence.cpp
SRCS_CPP += analyzer/analyzer_altitude_estimate_divergence.cpp
SRCS_CPP += analyzer/analyzer_attitude_estimate_divergence.cpp
SRCS_CPP += analyzer/analyzer_position_estimate_divergence.cpp
SRCS_CPP += analyzer/analyzer_vehicle_definition.cpp
SRCS_CPP += analyzervehicle_copter.cpp
SRCS_CPP += analyzervehicle_plane.cpp
SRCS_CPP += analyzervehicle.cpp
SRCS_CPP += la-log.cpp
SRCS_CPP += common_tool.cpp
SRCS_CPP += telem_client.cpp
SRCS_CPP += telem_forwarder_client.cpp
SRCS_CPP += telem_serial.cpp
SRCS_CPP += dataflash_logger.cpp 
SRCS_CPP += analyzing_dataflash_message_handler.cpp
SRCS_CPP += LA_MsgHandler.cpp
SRCS_CPP += analyzing_mavlink_message_handler.cpp
SRCS_C = util.c ini.c

OBJS = $(SRCS_CPP:.cpp=.o) $(SRCS_C:.c=.o)

DATAFLASH_LOGGER = dataflash_logger

LOG_ANALYZER = dronekit-la

IMAGETAGGER = imagetagger

all: $(LOG_ANALYZER)

$(DATAFLASH_LOGGER): $(OBJS) dataflash_logger_program.cpp
	$(LINK.cpp) -o $(DATAFLASH_LOGGER) dataflash_logger_program.cpp $(OBJS) $(LIBS) $(DLIBS)

$(LOG_ANALYZER): $(OBJS) loganalyzer.cpp
	$(LINK.cpp) -o $(LOG_ANALYZER) loganalyzer.cpp $(OBJS) $(LIBS) $(DLIBS)

$(IMAGETAGGER): $(OBJS) imagetagger.cpp mh_imagetagger.cpp
	$(LINK.cpp) -o $(IMAGETAGGER) imagetagger.cpp mh_imagetagger.cpp $(OBJS) $(LIBS) $(DLIBS)

clean:
	$(RM) *.o *~ $(DATAFLASH_LOGGER) $(LOG_ANALYZER) $(IMAGETAGGER) analyzer/*.o

test: clean all
	./test/test.sh

.PHONY: clean

install-dronekit-la: dronekit-la
	install -D dronekit-la $(DESTDIR)/usr/bin/dronekit-la

install: install-dronekit-la

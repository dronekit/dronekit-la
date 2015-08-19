
CFLAGS += -Wall $(INCS)

SRCS_C = utilTest.c util.c

OBJS = $(SRCS_C:.c=.o)

MAIN = utilTest

all: $(MAIN)

$(MAIN): $(OBJS)
	$(LINK.cpp) -o $(MAIN) $(OBJS) $(LIBS)

clean:
	$(RM) *.o *~ $(MAIN)

.PHONY: clean

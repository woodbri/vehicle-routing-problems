
CPP = g++
UTIL = ../baseClasses
UTILP = ../baseTrash
CPPFLAGS = -g -O0 -MMD  -MP  -I$(UTIL) -I$(UTILP)


SRCS = $(wildcard *.cpp) 
OBJS = $(SRCS:.cpp=.o)
UTILSRCS = $(wildcard $(UTIL)/*.cpp)
UTILOBJS = $(UTILSRCS:.cpp=.o)
DEPS = $(SRCS:.cpp=.d)
UTIL_HEADS = ../$(UTIL)/twpath.h ../$(UTIL)/twc.h ../$(UTIL)/twbucket.h ../$(UTIL)/plot.h



all: $(OBJS)


%.o: %.cpp $(UTIL_HEADS) 
	$(CPP) $(CPPFLAGS) -c $< -o $@



.PHONY: clean

clean:
	rm -f $(SRCS:.cpp=.o) $(DEPS) out/*.png ./*.png
	$(MAKE) -C $(UTIL) clean

-include $(DEPS)


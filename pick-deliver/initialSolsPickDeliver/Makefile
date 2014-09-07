
CPP = g++
UTILP = ../basePickDeliver
UTIL = ../../baseClasses
CPPFLAGS = -g -O0 -MMD  -MP  -I$(UTIL) -I$(UTILP)
LDFLAGS = -lgd -I$(UTIL) -I$(UTILP) -L$(UTIL) -L$(UTILP)


SRCS = $(wildcard *.cpp) 
OBJS = $(SRCS:.cpp=.o)
UTIL_SRCS = $(wildcard $(UTIL)/*.cpp)
UTIL_OBJS = $(UTIL_SRCS:.cpp=.o)
UTILP_SRCS = $(wildcard $(UTILP)/*.cpp)
UTILP_OBJS = $(UTILP_SRCS:.cpp=.o)
DEPS = $(SRCS:.cpp=.d)


all: tester

$(UTILP)/%.o: $(UTILP)/%.cpp
	$(MAKE) -C $(UTILP)

$(UTIL)/%.o: $(UTIL)/%.cpp
	$(MAKE) -C $(UTIL)

tester: $(UTIL_OBJS) $(OBJS) $(UTILP_OBJS)
	$(CPP) $^ -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CPP) $(CPPFLAGS) -c $< -o $@

test: tester lc008.txt
	./tester lc008.txt

valgrind: tester lc008.txt 
	valgrind -v --track-origins=yes --leak-check=full ./tester p50.txt

.PHONY: clean

clean:
	rm -f tester $(SRCS:.cpp=.o) $(DEPS) out/*.png ./*.png
	$(MAKE) -C $(UTIL) clean
	$(MAKE) -C $(UTILP) clean

-include $(DEPS)


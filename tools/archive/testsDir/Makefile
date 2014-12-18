
CPP = g++
UTIL = ../baseClasses
CPPFLAGS = -g -O0 -MMD -MP -I$(UTIL)
LDFLAGS = -lgd -I$(UTIL) -L$(UTIL)


SRCS = $(wildcard *.cpp) 
OBJS = $(SRCS:.cpp=.o)
UTILSRCS = $(wildcard $(UTIL)/*.cpp)
UTILOBJS = $(UTILSRCS:.cpp=.o)
DEPS = $(SRCS:.cpp=.d)


all: tester

$(UTIL)/%.o: $(UTIL)/%.cpp
	$(MAKE) -C $(UTIL)

tester: $(UTILOBJS) $(OBJS)
	$(CPP) $^ -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CPP) $(CPPFLAGS) -c $< -o $@

test: tester p50.txt
	./tester p50.txt

valgrind: tester p50.txt
	valgrind -v --track-origins=yes --leak-check=full ./tester p50.txt

.PHONY: clean

clean:
	rm -f tester $(SRCS:.cpp=.o) $(DEPS) out/*.png
	$(MAKE) -C $(UTIL) clean

-include $(DEPS)

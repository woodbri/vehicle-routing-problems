
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

small: tester p10.txt
	./tester p10.txt

valgrind: tester p50.txt
	valgrind -v --track-origins=yes --leak-check=full ./tester p50.txt

callgrind-50: tester p50.txt
	valgrind --tool=callgrind --callgrind-out-file=callgrind-50.out ./tester p50.txt
	callgrind_annotate callgrind-50.out | tee callgrind-50.txt

callgrind-1000: tester p1002.txt
	valgrind --tool=callgrind --callgrind-out-file=callgrind-1000.out ./tester p1002.txt
	callgrind_annotate callgrind-1000.out | tee callgrind-1000.txt

.PHONY: help

help:
	@echo "make <target>"
	@echo "  all - builds tester"
	@echo "  help - this message"
	@echo "  tester - builds tester"
	@echo "  test - runs: ./tester p50.txt"
	@echo "  small - runs: ./tester p10.txt"
	@echo "  valgrind - runs valgrind using p50.txt"
	@echo "  callgrind-50 - runs callgrind using p50.txt"
	@echo "  callgrind-1000 - runs callgrind using p1002.txt"
	@echo "  clean - cleans the build directories"

.PHONY: clean

clean:
	rm -f tester *.o *.d out/*.png
	$(MAKE) -C $(UTIL) clean

-include $(DEPS)

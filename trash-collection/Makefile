
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

small: tester p10.txt
	./tester trash p10.txt

test: tester p50.txt
	./tester trash p50.txt

medium: tester p1001.txt
	./tester trash p1001.txt

large: tester p5001.txt
	./tester trash p5001.txt

#      trash        - run the trash collection problem
#      swap         - test swap nodes between routes
#      2opt         - test 2-opt optimization of a route
#      movesbetween - test moves between multiple routes
#      moveswithin  - test moves within a route via e_*
#      moveswithinnew  - test moves within a route via vehicle
#      dumb         - test dump construction
#      nearestnode  - test findNearestNode
#      optmoves     - test various optimization moves

test-all: tester p50.txt
	./tester trash p50.txt > tests/a
	./tester swap jj >> tests/a
	./tester 2opt jj >> tests/a
	./tester movesbetween jj >> tests/a
	./tester moveswithin jj >> tests/a
	./tester moveswithinnew jj >> tests/a
	./tester dumb jj >> tests/a
	./tester nearestnode jj >> tests/a
	./tester optmoves jj >> tests/a
	diff tests/a tests/tests-all.txt
	rm tests/a


valgrind: tester p50.txt
	valgrind -v --track-origins=yes --leak-check=full ./tester trash p50.txt

callgrind-50: tester p50.txt
	valgrind --tool=callgrind --callgrind-out-file=callgrind-50.out ./tester trash p50.txt
	callgrind_annotate callgrind-50.out | tee callgrind-50.txt

callgrind-1000: tester p1002.txt
	valgrind --tool=callgrind --callgrind-out-file=callgrind-1000.out ./tester trash p1002.txt
	callgrind_annotate callgrind-1000.out | tee callgrind-1000.txt

.PHONY: help

help:
	@echo "make <target>"
	@echo "  all - builds tester"
	@echo "  help - this message"
	@echo "  tester - builds tester"
	@echo "  test - runs: ./tester trash p50.txt"
	@echo "  small - runs: ./tester trash p10.txt"
	@echo "  valgrind - runs valgrind using p50.txt"
	@echo "  callgrind-50 - runs callgrind using p50.txt"
	@echo "  callgrind-1000 - runs callgrind using p1002.txt"
	@echo "  clean - cleans the build directories"

.PHONY: clean

clean:
	rm -f tester *.o *.d out/*.png
	$(MAKE) -C $(UTIL) clean

-include $(DEPS)

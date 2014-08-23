
CPP = g++
CPPFLAGS = -g -O0 -MMD -MP 
LDFLAGS = -lgd

SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:.cpp=.o)
DEPS = $(SRCS:.cpp=.d)


all: $(OBJS) 

tester: $(OBJS)
	$(CPP) $^ -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CPP) $(CPPFLAGS) -c $< -o $@

test: tester p50.txt
	./tester p50.txt

valgrind: tester p50.txt
	valgrind -v --track-origins=yes --leak-check=full ./tester p50.txt

.PHONY: clean

clean:
	rm -f tester *.o *.d out/*.png

-include $(DEPS)

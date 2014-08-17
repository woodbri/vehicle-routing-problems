
CPP = g++
CPPFLAGS = -g -O0 -MMD -MP
LDFLAGS = -lgd

SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:.cpp=.o)
DEPS = $(SRCS:.cpp=.d)


all: vrpdptw

vrpdptw: $(OBJS)
	$(CPP) $^ -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CPP) $(CPPFLAGS) -c $< -o $@

test: vrpdptw lc101.txt
	./vrpdptw lc101.txt

valgrind: vrpdptw lc101.txt
	valgrind -v --track-origins=yes --leak-check=full ./vrpdptw lc101.txt

.PHONY: clean

clean:
	rm -f vrpdptw $(OBJS) $(DEPS) out/*.png

-include $(DEPS)

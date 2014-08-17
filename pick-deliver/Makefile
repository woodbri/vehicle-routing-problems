
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

test: vrpdptw lc008.txt lc032.txt lc101.txt
	./vrpdptw lc008.txt > lc008.result
	./vrpdptw lc032.txt > lc032.result
	./vrpdptw lc101.txt > lc101.result

small: vrpdptw lc008.txt
	./vrpdptw lc008.txt 

med: vrpdptw lc032.txt
	./vrpdptw lc032.txt 

big: vrpdptw lc101.txt
	./vrpdptw lc101.txt 

valgrind: vrpdptw lc008.txt 
	valgrind -v --track-origins=yes --leak-check=full ./vrpdptw lc101.txt

.PHONY: clean

clean:
	rm -f vrpdptw $(OBJS) $(DEPS) *.png out/*.png *.result

-include $(DEPS)


CPP = g++
CPPFLAGS = -g -O0 -MMD -MP
LDFLAGS = -lgd

SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:.cpp=.o)
DEPS = $(SRCS:.cpp=.d)


all: collection

collection: $(OBJS)
	$(CPP) $^ -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CPP) $(CPPFLAGS) -c $< -o $@

test: collection p50.txt
	./collection p50.txt

valgrind: collection p50.txt
	valgrind -v --track-origins=yes --leak-check=full ./collection p50.txt

.PHONY: clean

clean:
	rm -f collection $(OBJS) $(DEPS) out/*.png

-include $(DEPS)

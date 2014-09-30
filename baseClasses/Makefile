
CPP = g++
CPPFLAGS = -g -O0 -MMD -MP 
LDFLAGS = -lgd

SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:.cpp=.o)
DEPS = $(SRCS:.cpp=.d)


all: $(OBJS) 


%.o: %.cpp
	$(CPP) $(CPPFLAGS) -c $< -o $@



.PHONY: clean

clean:
	rm -f tester *.o *.d out/*.png

-include $(DEPS)

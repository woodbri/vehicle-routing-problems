
CPP = g++

CURL_CFLAGS := $(shell pkg-config --cflags curlpp)
CURL_LIBS := $(shell pkg-config --libs curlpp)
JSON_LIBS = -L /lib/x86_64-linux-gnu/ -ljson

CPPFLAGS = -g -O0 -MMD -MP -I/usr/local/include $(CURL_CFLAGS)
LDFLAGS = -lgd $(CURL_LIBS) $(JSON_LIBS)

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


CPP = g++
UTIL = ..

CURL_CFLAGS := $(shell pkg-config --cflags curlpp)
CURL_LIBS := $(shell pkg-config --libs curlpp)
JSON_LIBS = -L /lib/x86_64-linux-gnu/ -ljson

#OSRMBACKEND_DIR = $(HOME)/work/osrm-backend/
#OSRMBACKEND_DIR = /home/fpacheco/workspace/idm/osrm-backend
OSRMBACKEND_DIR = /usr/local/include/osrm


OSRMCLIENT_CFLAGS = -DOSRMCLIENT -I$(OSRMBACKEND_DIR)
OSRMCLIENT_LIBS = -L/usr/local/lib -lboost_filesystem -lboost_iostreams -lboost_program_options -lboost_regex -lboost_system -lboost_thread -lpthread -lOSRM -lz -lrt -lboost_filesystem -lboost_iostreams -lboost_program_options -lboost_regex -lboost_system -lboost_thread -lpthread -lstxxl

CPPFLAGS = -g -O0 -std=c++11 -MMD -MP -I$(UTIL) $(CURL_CFLAGS) $(OSRMCLIENT_CFLAGS) -DNDEBUG -I/usr/local/include
LDFLAGS = -lgd -I$(UTIL) -L$(UTIL) $(CURL_LIBS) $(JSON_LIBS) $(OSRMCLIENT_LIBS) -L/usr/local/lib -lglog

SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:.cpp=.o)
UTILSRCS = $(wildcard $(UTIL)/*.cpp)
UTILOBJS = $(UTILSRCS:.cpp=.o)
DEPS = $(SRCS:.cpp=.d)


all: help

$(UTIL)/%.o: $(UTIL)/%.cpp
	$(MAKE) -C $(UTIL)

tester: $(UTILOBJS) tester.o
	$(CPP) $(CPPFLAGS) $^ -o $@ $(LDFLAGS)

testosrm: $(UTILOBJS) testosrm.o
	$(CPP) $(CPPFLAGS) $^ -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CPP) $(CPPFLAGS) -c $< -o $@

test: tester test-data.txt 
	./tester test-data.txt 

valgrind: tester test-data.txt
	valgrind -v --track-origins=yes --leak-check=full ./tester test-data.txt

callgrind: tester test-data.txt
	valgrind --tool=callgrind --callgrind-out-file=callgrind.out ./tester test-data.txt
	callgrind_annotate callgrind.out | tee callgrind.txt

.PHONY: help

help:
	@echo "make <target>"
	@echo "  help - this message"
	@echo "  tester - builds tester"
	@echo "  testeosrm - builds testosrm
	@echo "  test - runs: ./tester test-data.txt"
	@echo "  valgrind - runs valgrind using test-data.txt"
	@echo "  callgrind - runs callgrind using test-data.txt"
	@echo "  clean - cleans the build directories"

.PHONY: clean

clean:
	rm -f tester testosrm *.o *.d out/*.png
	$(MAKE) -C $(UTIL) clean

-include $(DEPS)


CPP = g++

#OSRMBACKEND_DIR = $(HOME)/work/osrm-backend/
#OSRMBACKEND_DIR = /home/fpacheco/workspace/idm/osrm-backend
OSRMBACKEND_DIR = /usr/local/include/osrm


CURL_CFLAGS := $(shell pkg-config --cflags curlpp)
CURL_LIBS := $(shell pkg-config --libs curlpp)
JSON_LIBS = -L /lib/x86_64-linux-gnu/ -ljson

CPPFLAGS = -g -O0 -std=c++11 -MMD -MP -I/usr/local/include -I.  -DVRPMINTRACE -DOSRMCLIENTTRACE -DDOVRPLOG 
LDFLAGS = -lgd 

SRCS = $(wildcard *.cpp)


WITH_OSRMCLIENT = yes
WITH_MD5 = yes 
WITH_STATS = yes
WITH_VRPOSRM = yes

ifeq ("$(WITH_VRPOSRM)", "")
OSRM_CFLAGS =
OSRM_LIBS =
OSRM_HEADER =
SRCS := $(filter-out vrposrm.cpp, $(SRCS))
else
OSRM_HEADER = md5.h
OSRM_CFLAGS = -DMD5 
OSRM_LIBS =
endif

ifeq ("$(WITH_MD5)", "")
MD5_CFLAGS =
MD5_LIBS =
MD5_HEADER =
SRCS := $(filter-out md5.cpp, $(SRCS))
else
MD5_HEADER = md5.h
MD5_CFLAGS = -DMD5 
MD5_LIBS =
endif

ifeq ("$(WITH_STATS)", "")
STATS_CFLAGS =
STATS_LIBS =
STATS_HEADER =
SRCS := $(filter-out stats.cpp, $(SRCS))
SRCS := $(filter-out timer.cpp, $(SRCS))
else
STATS_HEADER = stats.h timer.h singleton.h
STATS_CFLAGS = -DDOSTATS
STATS_LIBS =
endif


ifeq ("$(WITH_OSRMCLIENT)", "")
OSRMCLIENT_CFLAGS =
OSRMCLIENT_LIBS =
OSRMCLIENT_HEADER = 
SRCS := $(filter-out osrmclient.cpp, $(SRCS))
else
OSRMCLIENT_HEADER =../../$(BASECLASSES)/osrmclient.h
OSRMCLIENT_CFLAGS = -DOSRMCLIENT -I$(OSRMBACKEND_DIR)
OSRMCLIENT_LIBS = -L/usr/local/lib -lboost_filesystem -lboost_iostreams -lboost_program_options -lboost_regex -lboost_system -lboost_thread -lpthread -lOSRM -lz -lrt -lboost_filesystem -lboost_iostreams -lboost_program_options -lboost_regex -lboost_system -lboost_thread -lpthread -lstxxl  $(JSON_LIBS)
endif


OBJS = $(SRCS:.cpp=.o)
DEPS = $(SRCS:.cpp=.d)


all: $(OBJS) 

	


config.o: config.cpp config.h singleton.h
	$(CPP) $(CPPFLAGS) -c $< -o $@

md5.o: md5.cpp md5.h
	$(CPP) $(CPPFLAGS) -c $< -o $@

move.o: move.cpp move.h
	$(CPP) $(CPPFLAGS) -c $< -o $@

node.o: node.cpp node.h
	$(CPP) $(CPPFLAGS) -c $< -o $@

twnode.o: twnode.cpp twnode.h node.h
	$(CPP) $(CPPFLAGS) -c $< -o $@

tweval.o: tweval.cpp tweval.h twnode.h node.h
	$(CPP) $(CPPFLAGS) -c $< -o $@

stats.o: stats.cpp $(STATS_HEADER)
	$(CPP) $(CPPFLAGS) -c $< -o $@

osrmclient.o: osrmclient.cpp osrmclient.h node.h $(STATS_HEADER)
	$(CPP) $(CPPFLAGS) $(OSRMCLIENT_CFLAGS) -c $< -o $@

%.o: %.cpp
	$(CPP) $(CPPFLAGS) -c $< -o $@



.PHONY: clean

clean:
	rm -f tester *.o *.d out/*.png

-include $(DEPS)

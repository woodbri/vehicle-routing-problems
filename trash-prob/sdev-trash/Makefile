
CPP = g++

CURL_CFLAGS := $(shell pkg-config --cflags curlpp)
CURL_LIBS := $(shell pkg-config --libs curlpp)
JSON_LIBS = -L /lib/x86_64-linux-gnu/ -ljson

BASETRASH = baseTrash
BASECLASSES = baseClasses
INITTRASH = initTrash
SOLVERTRASH = solverTrash

BUILD_DIR = ./build
BIN_DIR = ./bin

# set WITH_OSRMCLIENT = "1" to include this code
# you will also need to define additional related variables

WITH_OSRMCLIENT = "1"

ifeq ($(WITH_OSRMCLIENT), "")
OSRM_CFLAGS =
OSRM_LIBS =
else
OSRM_CFLAGS = -DOSRMCLIENT -I$(HOME)/work/osrm-backend/
OSRM_LIBS = -L/usr/local/lib -lboost_filesystem -lboost_iostreams -lboost_program_options -lboost_regex -lboost_system -lboost_thread -lpthread -lOSRM -lUUID -lGITDESCRIPTION -lz -lrt -lboost_filesystem -lboost_iostreams -lboost_program_options -lboost_regex -lboost_system -lboost_thread -lpthread -lstxxl
endif



INCLUDES = -I../../$(BASECLASSES) -I../../$(BASETRASH) -I../../$(INITTRASH) -I../../$(SOLVERTRASH) -I.
CPPFLAGS = -g -O0 -MMD  -MP $(CURL_CFLAGS) $(OSRM_CFLAGS) $(DEFINES) -I/usr/local/include
LDFLAGS = -lgd $(INCLUDES) -L$(BASECLASSES) -L$(BASETRASH) -L$(INITTRASH) $(CURL_LIBS) $(JSON_LIBS) $(OSRM_LIBS) -L/usr/local/lib -lglog

SRCS =  
OBJS = $(addprefix $(BUILD_DIR)/,$(SRCS:.cpp=.o))
HEADS = $(SRCS:.cpp=.h)



BASECLASSES_SRCS = $(wildcard ../../$(BASECLASSES)/*.cpp)

# filter out baseClasses/osrmclient.cpp unless
# WITH_OSRMCLIENT=1 has been set

ifeq ($(WITH_OSRMCLIENT), "")
BASECLASSES_SRCS := $(filter-out ../../$(BASECLASSES)/osrmclient.cpp, $(BASECLASSES_SRCS))
endif

BASECLASSES_HDS = $(UTIL_SRCS:.cpp=.h)
BASECLASSES_OBJS = $(addprefix $(BUILD_DIR)/$(BASECLASSES)/,$(notdir $(BASECLASSES_SRCS:.cpp=.o)))
BASECLASSES_HEADS = ../../$(BASECLASSES)/twpath.h ../../$(BASECLASSES)/twc.h ../../$(BASECLASSES)/twbucket.h ../../$(BASECLASSES)/plot.h ../../$(BASECLASSES)/singleton.h ../../$(BASECLASSES)/timer.h ../../$(BASECLASSES)/logger.h $(BASECLASSES_HDS)


BASETRASH_SRCS = $(wildcard ../../$(BASETRASH)/*.cpp)
BASETRASH_HDS = $(BASETRASH_SRCS:.cpp=.h) ../../$(BASETRASH)/trashconfig.h
BASETRASH_OBJS = $(addprefix $(BUILD_DIR)/$(BASETRASH)/,$(notdir $(BASETRASH_SRCS:.cpp=.o)))
BASETRASH_HEADS = $(BASETRASH_HDS) $(BASECLASSES_HEADS)

INITTRASH_SRCS = $(wildcard ../../$(INITTRASH)/*.cpp)
INITTRASH_HDS = $(INITTRASH_SRCS:.cpp=.h)
INITTRASH_OBJS = $(addprefix $(BUILD_DIR)/$(INITTRASH)/,$(notdir $(INITTRASH_SRCS:.cpp=.o)))
INITTRASH_HEADS = $(INITTRASH_HDS)  $(BASECLASSES_HEADS)

SOLVERTRASH_SRCS = $(wildcard ../../$(SOLVERTRASH)/*.cpp)
SOLVERTRASH_HDS = $(SOLVERTRASH_SRCS:.cpp=.h)
SOLVERTRASH_OBJS = $(addprefix $(BUILD_DIR)/$(SOLVERTRASH)/,$(notdir $(SOLVERTRASH_SRCS:.cpp=.o)))
SOLVERTRASH_HEADS = $(SOLVERTRASH_HDS)  $(BASECLASSES_HEADS)



DEPS = $(addprefix $(BUILD_DIR)/,$(SRCS:.cpp=.d))




all:   $(OBJS) $(BASECLASSES_OBJS) $(BASETRASH_OBJS) $(INITTRASH_OBJS) $(SOLVERTRASH_OBJS) build/trash.o ./bin/trash



build/trash.o: trash.cpp $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS) $(SOLVERTRASH_HDS) $(HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

build/baseClasses/%.o: ../../$(BASECLASSES)/%.cpp  ../../$(BASECLASSES)/%.h $(BASECLASSES_HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

build/baseTrash/%.o: ../../$(BASETRASH)/%.cpp ../../$(BASETRASH)/%.h  $(BASECLASSES_HEADS) $(BASETRASH_HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@


build/initTrash/%.o: ../../$(INITTRASH)/%.cpp  ../../$(INITTRASH)/%.h  $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

build/solverTrash/%.o: ../../$(SOLVERTRASH)/%.cpp  ../../$(SOLVERTRASH)/%.h  $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(SOLVERTRASH_HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

build/%.o: %.cpp %.h $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS) $(SOLVERTRASH_HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

bin/trash: build/trash.o $(OBJS) $(BASECLASSES_OBJS) $(BASETRASH_OBJS) $(INITTRASH_OBJS) $(SOLVERTRASH_OBJS) $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS) $(SOLVERTRASH_HEADS)
	$(CPP) $^ -o $@ $(LDFLAGS)

test: ./bin/trash ./montevideo/muni-b.containers.txt ./montevideo/muni-b.otherlocs.txt ./montevideo/muni-b.dmatrix-time.txt ./montevideo/muni-b.vehicles.txt
	./bin/trash ./montevideo/muni-b >./logs/muni-b.out

valgrind: ./bin/trash ./montevideo/muni-b.containers.txt ./montevideo/muni-b.otherlocs.txt ./montevideo/muni-b.dmatrix-time.txt ./montevideo/muni-b.vehicles.txt
	valgrind -v --track-origins=yes --leak-check=full ./bin/trash montevideo/muni-b

callgrind-8: ./bin/trash ./montevideo/muni-b.*
	valgrind --tool=callgrind --callgrind-out-file=logs/callgrind-008.out ./bin/trash montevideo/muni-b
	callgrind_annotate logs/callgrind-008.out | tee logs/callgrind-008.txt


.PHONY: clean

clean:
	rm -rf $(BIN_DIR)/trash $(BUILD_DIR)/*
	find . -name "*.d" -type f -delete
	find . -name "*.png" -type f -delete
	mkdir -p $(BIN_DIR)
	echo $(INITTRASH)
	mkdir -p $(BUILD_DIR)/$(BASECLASSES) $(BUILD_DIR)/$(BASETRASH) $(BUILD_DIR)/$(INITTRASH) $(BUILD_DIR)/$(SOLVERTRASH)


cleanAll: clean
	$(MAKE) -C ../../$(BASECLASSES) clean
	$(MAKE) -C ../../$(BASETRASH) clean
	$(MAKE) -C ../../$(INITTRASH) clean
	$(MAKE) -C ../../$(SOLVERTRASH) clean
	rm ./bin/trash

-include $(DEPS)


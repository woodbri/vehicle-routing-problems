
AR = ar
RANLIB = ranlib
CPP = g++




CPP = g++

# when blank, even when set to compile with OSRM it will compile without osrm
#OSRMBACKEND_DIR = $(HOME)/work/osrm-backend/
#OSRMBACKEND_DIR = /home/fpacheco/workspace/idm/osrm-backend
OSRMBACKEND_DIR = /usr/local/include/osrm
 
BASETRASH = baseTrash
BASECLASSES = baseClasses
INITTRASH = initTrash
SOLVERTRASH = solverTrash

BASECLASSES_DIR = ../../../$(BASECLASSES)
BASETRASH_DIR=../../$(BASETRASH)
INITTRASH_DIR=../../$(INITTRASH)
SOLVERTRASH_DIR=../$(SOLVERTRASH)

BUILD_DIR = ./build
BIN_DIR = ./bin

# setting NO_TRACE = <something>  invalidates the other traces
NO_TRACE =
TRACE_MIN = yes
TRACE_ALL = 

ifeq ("$(NO_TRACE)", "")
  ifeq ("$(TRACE_ALL)", "")
    ifeq ("$(TRACE_MIN)", "")
      TRACE_CFLAGS =
    else
      TRACE_CFLAGS = -DVRPMINTRACE -DOSRMCLIENTTRACE
    endif
  else
    TRACE_CFLAGS = -DVRPMINTRACE -DVRPMAXTRACE -DOSRMCLIENTTRACE
  endif
else
  TRACE_CFLAGS =
endif




# <blank> is NO
# anything else  

WITH_OSRMCLIENT = yes
WITH_STATS = yes
WITH_MD5 =
WITH_OSRM =
WITH_PLOT =
DEBUG_MODE = yes

ifeq ("$(DEBUG_MODE)", "")
else
WITH_LOG = yes
endif

ifeq ("$(WITH_STATS)", "")
else
WITH_LOG = yes
endif

ifeq ("$(TRACE_CFLAGS)", "")
else
WITH_LOG = yes
endif



ifeq ("$(DEBUG_MODE)", "")
DEBUG_CFLAGS = -DNDEBUG 
else
DEBUG_CFLAGS = 
endif

ifeq ("$(WITH_STATS)", "")
STATS_CFLAGS =
STATS_HEADER =
else
STATS_HEADER = $(BASECLASSES_DIR)/stats.h $(BASECLASSES_DIR)/timer.h $(BASECLASSES_DIR)/singleton.h
STATS_CFLAGS = -DDOSTATS
endif

ifeq ("$(WITH_OSRM)", "")
OSRM_CFLAGS =
OSRM_HEADER =
else
OSRM_HEADER = $(BASECLASSES_DIR)/vrposrm.h
OSRM_CFLAGS = -DWITHOSRM $(CURL_CFLAGS)
endif

ifeq ("$(WITH_PLOT)", "")
PLOT_HEADER =
PLOT_CFLAGS =
else
PLOT_HEADER = $(BASECLASSES_DIR)/plot.h
PLOT_CFLAGS = -DDOPLOT
endif


ifeq ("$(WITH_MD5)", "")
MD5_CFLAGS =
else
MD5_CFLAGS = -DWITHMD5
endif

ifeq ("$(WITH_LOG)", "")
LOGGER_HEADER =
LOGGER_CFLAGS =
LOGGER_LIBS =
else
LOGGER_HEADER = $(BASECLASSES_DIR)/logger.h
LOGGER_CFLAGS = -DDOVRPLOG -I/usr/local/include/
LOGGER_LIBS = -L/usr/local/lib -lglog
endif




ifeq ("$(WITH_OSRMCLIENT)", "")
OSRMCLIENT_CFLAGS =
OSRMCLIENT_LIBS =
OSRMCLIENT_HEADER =
else
OSRMCLIENT_HEADER =$(BASECLASSES_DIR)/osrmclient.h
OSRMCLIENT_CFLAGS = -std=c++11 -DOSRMCLIENT -I$(OSRMBACKEND_DIR)
OSRMCLIENT_LIBS = -L/usr/local/lib -lboost_filesystem -lboost_iostreams -lboost_program_options -lboost_regex \
	-lboost_system -lboost_thread -lpthread -lOSRM -lz -lrt \
	-lboost_filesystem -lboost_iostreams -lboost_program_options -lboost_regex -lboost_system -lboost_thread -lpthread -lstxxl
endif



INCLUDES = -I$(BASECLASSES_DIR) -I$(BASETRASH_DIR) -I$(INITTRASH_DIR) -I$(SOLVERTRASH_DIR) -I. 
CPPCOMPILEFLAGS = -Wreorder -Wall -Wextra -Wdouble-promotion 
CPPCOMPILEFLAGS =
CPPFLAGS = -g -O0 -MMD  -MP -fPIC $(CPPCOMPILEFLAGS) $(OSRMCLIENT_CFLAGS) $(DEFINES) $(STATS_CFLAGS) $(PLOT_CFLAGS) $(LOGGER_CFLAGS) $(DEBUG_CFLAGS) $(TRACE_CFLAGS)
LDFLAGS = -L/usr/lib/x86_64-linux-gnu -lgd $(INCLUDES) -L$(BASECLASSES) -L$(BASETRASH) -L$(INITTRASH)  $(JSON_LIBS) $(OSRMCLIENT_LIBS) $(LOGGER_LIBS)

CPPFLAGS_OUT = $(INCLUDES) $(OSRMCLIENT_CFLAGS) $(DEFINES) $(STATS_CFLAGS) $(PLOT_CFLAGS) $(LOGGER_CFLAGS) $(DEBUG_CFLAGS) $(TRACE_CFLAGS)
LDFLAGS_OUT = -L$(BASECLASSES) -L$(BASETRASH) -L$(INITTRASH)  $(OSRMCLIENT_LIBS) $(LOGGER_LIBS)




SRCS =
OBJS = $(addprefix $(BUILD_DIR)/,$(SRCS:.cpp=.o))
HEADS = $(SRCS:.cpp=.h)


BASECLASSES_SRCS = $(wildcard $(BASECLASSES_DIR)/*.cpp)
BASECLASSES_HDS = $(UTIL_SRCS:.cpp=.h)


ifeq ("$(WITH_OSRMCLIENT)", "")
BASECLASSES_SRCS := $(filter-out $(BASECLASSES_DIR)/osrmclient.cpp, $(BASECLASSES_SRCS))
BASECLASSES_HDS := $(filter-out $(BASECLASSES_DIR)/osrmclient.h, $(BASECLASSES_HDS))
endif

ifeq ("$(WITH_STATS)", "")
BASECLASSES_SRCS := $(filter-out $(BASECLASSES_DIR)/stats.cpp, $(BASECLASSES_SRCS))
BASECLASSES_SRCS := $(filter-out $(BASECLASSES_DIR)/timer.cpp, $(BASECLASSES_SRCS))
BASECLASSES_HDS := $(filter-out $(BASECLASSES_DIR)/stats.h, $(BASECLASSES_HDS))
BASECLASSES_HDS := $(filter-out $(BASECLASSES_DIR)/timer.h, $(BASECLASSES_HDS))
endif

ifeq ("$(WITH_MD5)", "")
BASECLASSES_SRCS := $(filter-out $(BASECLASSES_DIR)/md5.cpp, $(BASECLASSES_SRCS))
BASECLASSES_HDS := $(filter-out $(BASECLASSES_DIR)/md5.h, $(BASECLASSES_HDS))
endif

ifeq ("$(WITH_OSRM)", "")
BASECLASSES_SRCS := $(filter-out $(BASECLASSES_DIR)/vrposrm.cpp, $(BASECLASSES_SRCS))
BASECLASSES_HDS := $(filter-out $(BASECLASSES_DIR)/vrposrm.h, $(BASECLASSES_HDS))
endif

BASECLASSES_SRCS := $(filter-out $(BASECLASSES_DIR)/config.cpp, $(BASECLASSES_SRCS))
BASECLASSES_HDS := $(filter-out $(BASECLASSES_DIR)/config.h, $(BASECLASSES_HDS))

BASECLASSES_OBJS = $(addprefix $(BUILD_DIR)/$(BASECLASSES)/,$(notdir $(BASECLASSES_SRCS:.cpp=.o)))
BASECLASSES_HEADS = $(BASECLASSES_DIR)/twpath.h $(BASECLASSES_DIR)/twc.h $(BASECLASSES_DIR)/twbucket.h  $(BASECLASSES_DIR)/singleton.h $(BASECLASSES_HDS) $(PLOT_HEADER)





BASETRASH_SRCS = $(wildcard $(BASETRASH_DIR)/*.cpp)

BASETRASH_SRCS := $(filter-out $(BASETRASH_DIR)/trashconfig.cpp, $(BASETRASH_SRCS))
BASETRASH_HDS := $(filter-out $(BASETRASH_DIR)/trashconfig.h, $(BASETRASH_HDS))
BASETRASH_SRCS := $(filter-out $(BASETRASH_DIR)/extendedvehicle.cpp, $(BASETRASH_SRCS))
BASETRASH_HDS := $(filter-out $(BASETRASH_DIR)/extendedvehicle.h, $(BASETRASH_HDS))

BASETRASH_OBJS = $(addprefix $(BUILD_DIR)/$(BASETRASH)/,$(notdir $(BASETRASH_SRCS:.cpp=.o)))
BASETRASH_HEADS = $(BASETRASH_HDS) $(BASECLASSES_HEADS)



INITTRASH_SRCS =  $(INITTRASH_DIR)/truckManyVisitsDump.cpp $(INITTRASH_DIR)/feasableSolLoop.cpp  $(INITTRASH_DIR)/givenInitialSol.cpp
INITTRASH_HDS = $(INITTRASH_SRCS:.cpp=.h)
INITTRASH_OBJS = $(addprefix $(BUILD_DIR)/$(INITTRASH)/,$(notdir $(INITTRASH_SRCS:.cpp=.o)))
INITTRASH_HEADS = $(INITTRASH_HDS)  $(BASECLASSES_HEADS)

SOLVERTRASH_SRCS = $(wildcard $(SOLVERTRASH_DIR)/*.cpp)
SOLVERTRASH_HDS = $(SOLVERTRASH_SRCS:.cpp=.h)
SOLVERTRASH_OBJS = $(addprefix $(BUILD_DIR)/$(SOLVERTRASH)/,$(notdir $(SOLVERTRASH_SRCS:.cpp=.o)))
SOLVERTRASH_HEADS = $(SOLVERTRASH_HDS)  $(BASECLASSES_HEADS)



DEPS = $(addprefix $(BUILD_DIR)/,$(SRCS:.cpp=.d))




all:   $(OBJS) $(BASECLASSES_OBJS) $(BASETRASH_OBJS) $(INITTRASH_OBJS) $(SOLVERTRASH_OBJS) build/trash.o build/trash_given_sol.o ./build/libvrptools.a



build/trash.o: trash.cpp $(OSRMCLIENT_HEADER) $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS) $(SOLVERTRASH_HDS) $(HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

build/trash_given_sol.o: trash_given_sol.cpp $(OSRMCLIENT_HEADER) $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS) $(SOLVERTRASH_HDS) $(HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@


build/baseClasses/config.o: $(BASECLASSES_DIR)/config.cpp $(BASECLASSES_DIR)/config.h $(BASECLASSES_DIR)/singleton.h
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

build/baseClasses/move.o: $(BASECLASSES_DIR)/move.cpp $(BASECLASSES_DIR)/move.h $(BASECLASSES_DIR)/singleton.h $(STATS_HEADER)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

build/baseClasses/node.o: $(BASECLASSES_DIR)/node.cpp $(BASECLASSES_DIR)/node.h
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

build/baseClasses/stats.o: $(BASECLASSES_DIR)/stats.cpp $(STATS_HEADER)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@


build/baseClasses/%.o: $(BASECLASSES_DIR)/%.cpp  $(BASECLASSES_DIR)/%.h $(BASECLASSES_HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@



build/baseTrash/%.o: $(BASETRASH_DIR)/%.cpp $(BASETRASH_DIR)/%.h  $(BASECLASSES_HEADS) $(BASETRASH_HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@


build/initTrash/%.o: $(INITTRASH_DIR)/%.cpp  ../../$(INITTRASH)/%.h  $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

build/solverTrash/%.o: $(SOLVERTRASH_DIR)/%.cpp  $(SOLVERTRASH_DIR)/%.h  $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(SOLVERTRASH_HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

build/%.o: %.cpp %.h $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS) $(SOLVERTRASH_HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

bin/trash: build/trash.o $(OBJS) $(BASECLASSES_OBJS) $(BASETRASH_OBJS) $(INITTRASH_OBJS) $(SOLVERTRASH_OBJS) $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS) $(SOLVERTRASH_HEADS)
	$(CPP) $(CPPFLAGS_OUT) $^ -o $@ $(LDFLAGS)

bin/trash_given_sol: build/trash_given_sol.o $(OBJS) $(BASECLASSES_OBJS) $(BASETRASH_OBJS) $(INITTRASH_OBJS) $(SOLVERTRASH_OBJS) $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS) $(SOLVERTRASH_HEADS)
	$(CPP) $(CPPFLAGS_OUT) $^ -o $@ $(LDFLAGS)

TESTPREFIX =  ../tests/ch_du_rm_cl_05
TESTFILES =  $(TESTPREFIX).containers.txt $(TESTPREFIX).otherlocs.txt $(TESTPREFIX).dmatrix-time.txt $(TESTPREFIX).vehicles.txt


test: ./bin/trash  $(TESTFILES)
	./bin/trash  $(TESTPREFIX)  > $(TESTPREFIX).out.txt

test1: ./bin/trash_given_sol ../muni-b/muni-b.containers.txt ../muni-b/muni-b.otherlocs.txt ../muni-b/muni-b.dmatrix-time.txt ../muni-b/muni-b.vehicles.txt
	./bin/trash_given_sol ../muni-b/muni-b >./logs/muni-b.out

build/libvrptools.a: $(OBJS) $(BASECLASSES_OBJS) $(BASETRASH_OBJS) $(INITTRASH_OBJS) $(SOLVERTRASH_OBJS) $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS) $(SOLVERTRASH_HEADS)
	$(AR) -cq $@ $(OBJS) $(BASECLASSES_OBJS) $(BASETRASH_OBJS) $(INITTRASH_OBJS) $(SOLVERTRASH_OBJS)
	$(RANLIB) $(@)
	@echo VRP_HEADERS = $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS) $(SOLVERTRASH_HEADS) > $(BUILD_DIR)/vrptools-mk.inc
	@echo VRP_CFLAGS = $(CPPFLAGS_OUT) >> $(BUILD_DIR)/vrptools-mk.inc
	@echo VRP_LDFLAGS = $(LDFLAGS_OUT) >> $(BUILD_DIR)/vrptools-mk.inc





valgrind: 
	@echo "Output will be in valgrindlog"
	valgrind -v --track-origins=yes --undef-value-errors=yes --leak-check=full --show-leak-kinds=all ./bin/trash ../muni-b/muni-b &> logs/valgrindlog

callgrind-8: ./bin/trash ./montevideo/muni-b.*
	valgrind --tool=callgrind --callgrind-out-file=logs/callgrind-008.out ./bin/trash montevideo/muni-b
	callgrind_annotate logs/callgrind-008.out | tee logs/callgrind-008.txt



.PHONY: clean

clean:
	rm -rf $(BIN_DIR)/trash $(BUILD_DIR)/*
	find . -name "*.d" -type f -delete
	find . -name "*.png" -type f -delete
	rm -f *.o
	rm -f *.so
	mkdir -p $(BIN_DIR)
	echo $(INITTRASH)
	mkdir -p $(BUILD_DIR)/$(BASECLASSES) $(BUILD_DIR)/$(BASETRASH) $(BUILD_DIR)/$(INITTRASH) $(BUILD_DIR)/$(SOLVERTRASH) ./logs


cleanAll: clean
	$(MAKE) -C $(BASECLASSES_DIR) clean
	$(MAKE) -C $(BASETRASH_DIR) clean
	$(MAKE) -C $(INITTRASH_DIR) clean
	$(MAKE) -C $(SOLVERTRASH_DIR) clean
	rm ./bin/trash

-include $(DEPS)



AR = ar
RANLIB = ranlib
CPP = g++




CPP = g++



BASETRASH = baseTrash
BASECLASSES = baseClasses
INITTRASH = initTrash
SOLVERTRASH = solverTrash

BASECLASSES_DIR = ../../$(BASECLASSES)
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
TRACE_CFLAGS = -DVRPMINTRACE
endif
else
TRACE_CFLAGS = -DVRPMINTRACE -DVRPMAXTRACE
endif
else
TRACE_CFLAGS =
endif




# <blank> is NO
# anything else  

WITH_OSRMCLIENT = yes
WITH_STATS =
WITH_MD5 =
WITH_OSRM =
WITH_PLOT =
DEBUG_MODE =

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
LOGGER_CFLAGS = -DDOVRPLOG -I/usr/local/include
LOGGER_LIBS = -L/usr/local/lib -lglog
endif




ifeq ("$(WITH_OSRMCLIENT)", "")
OSRMCLIENT_CFLAGS =
OSRMCLIENT_LIBS =
OSRMCLIENT_HEADER =
else
OSRMCLIENT_HEADER =../../$(BASECLASSES)/osrmclient.h
OSRMCLIENT_CFLAGS = -DOSRMCLIENT -I$(HOME)/work/osrm-backend/
OSRMCLIENT_LIBS = -L/usr/local/lib -lboost_filesystem -lboost_iostreams -lboost_program_options -lboost_regex -lboost_system -lboost_thread -lpthread -lOSRM -lUUID -lGITDESCRIPTION -lz -lrt -lboost_filesystem -lboost_iostreams -lboost_program_options -lboost_regex -lboost_system -lboost_thread -lpthread -lstxxl
endif



INCLUDES = -I../../$(BASECLASSES) -I../../$(BASETRASH) -I../../$(INITTRASH) -I../../$(SOLVERTRASH) -I.
CPPFLAGS = -g -O0 -MMD  -MP -fPIC $(OSRMCLIENT_CFLAGS) $(DEFINES) $(STATS_CFLAGS) $(PLOT_CFLAGS) $(LOGGER_CFLAGS) $(DEBUG_CFLAGS) $(TRACE_CFLAGS)
LDFLAGS = -lgd $(INCLUDES) -L$(BASECLASSES) -L$(BASETRASH) -L$(INITTRASH)  $(JSON_LIBS) $(OSRMCLIENT_LIBS) $(LOGGER_LIBS)

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


BASECLASSES_OBJS = $(addprefix $(BUILD_DIR)/$(BASECLASSES)/,$(notdir $(BASECLASSES_SRCS:.cpp=.o)))
BASECLASSES_HEADS = ../../$(BASECLASSES)/twpath.h ../../$(BASECLASSES)/twc.h ../../$(BASECLASSES)/twbucket.h  ../../$(BASECLASSES)/singleton.h $(BASECLASSES_HDS) $(PLOT_HEADER)





BASETRASH_SRCS = $(wildcard ../../$(BASETRASH)/*.cpp)
BASETRASH_HDS = $(BASETRASH_SRCS:.cpp=.h) ../../$(BASETRASH)/trashconfig.h
BASETRASH_OBJS = $(addprefix $(BUILD_DIR)/$(BASETRASH)/,$(notdir $(BASETRASH_SRCS:.cpp=.o)))
BASETRASH_HEADS = $(BASETRASH_HDS) $(BASECLASSES_HEADS)

INITTRASH_SRCS =  ../../$(INITTRASH)/feasableSol.cpp
INITTRASH_HDS = $(INITTRASH_SRCS:.cpp=.h)
INITTRASH_OBJS = $(addprefix $(BUILD_DIR)/$(INITTRASH)/,$(notdir $(INITTRASH_SRCS:.cpp=.o)))
INITTRASH_HEADS = $(INITTRASH_HDS)  $(BASECLASSES_HEADS)

SOLVERTRASH_SRCS = $(wildcard ../../$(SOLVERTRASH)/*.cpp)
SOLVERTRASH_HDS = $(SOLVERTRASH_SRCS:.cpp=.h)
SOLVERTRASH_OBJS = $(addprefix $(BUILD_DIR)/$(SOLVERTRASH)/,$(notdir $(SOLVERTRASH_SRCS:.cpp=.o)))
SOLVERTRASH_HEADS = $(SOLVERTRASH_HDS)  $(BASECLASSES_HEADS)



DEPS = $(addprefix $(BUILD_DIR)/,$(SRCS:.cpp=.d))




all:   $(OBJS) $(BASECLASSES_OBJS) $(BASETRASH_OBJS) $(INITTRASH_OBJS) $(SOLVERTRASH_OBJS) build/trash.o ./bin/trash ./build/libvrptools.a



build/trash.o: trash.cpp $(OSRMCLIENT_HEADER) $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS) $(SOLVERTRASH_HDS) $(HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@


build/baseClasses/config.o: $(BASECLASSES_DIR)/config.cpp $(BASECLASSES_DIR)/config.h $(BASECLASSES_DIR)/singleton.h
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

build/baseClasses/move.o: $(BASECLASSES_DIR)/move.cpp $(BASECLASSES_DIR)/move.h $(BASECLASSES_DIR)/singleton.h $(STATS_HEADER)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

build/baseClasses/node.o: $(BASECLASSES_DIR)/node.cpp $(BASECLASSES_DIR)/node.h
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

build/baseClasses/stats.o: $(BASECLASSES_DIR)/stats.cpp $(STATS_HEADER)
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


build/libvrptools.a: $(OBJS) $(BASECLASSES_OBJS) $(BASETRASH_OBJS) $(INITTRASH_OBJS) $(SOLVERTRASH_OBJS) $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS) $(SOLVERTRASH_HEADS)
	$(AR) -cq $@ $(OBJS) $(BASECLASSES_OBJS) $(BASETRASH_OBJS) $(INITTRASH_OBJS) $(SOLVERTRASH_OBJS)
	$(RANLIB) $(@)
	@echo VRP_HEADERS = $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS) $(SOLVERTRASH_HEADS) > $(BUILD_DIR)/vrptools-mk.inc
	@echo VRP_CFLAGS = $(CPPFLAGS_OUT) >> $(BUILD_DIR)/vrptools-mk.inc
	@echo VRP_LDFLAGS = $(LDFLAGS_OUT) >> $(BUILD_DIR)/vrptools-mk.inc





valgrind: 
	valgrind -v --track-origins=yes --leak-check=full ./bin/trash montevideo/muni-b

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
	mkdir -p $(BUILD_DIR)/$(BASECLASSES) $(BUILD_DIR)/$(BASETRASH) $(BUILD_DIR)/$(INITTRASH) $(BUILD_DIR)/$(SOLVERTRASH)


cleanAll: clean
	$(MAKE) -C ../../$(BASECLASSES) clean
	$(MAKE) -C ../../$(BASETRASH) clean
	$(MAKE) -C ../../$(INITTRASH) clean
	$(MAKE) -C ../../$(SOLVERTRASH) clean
	rm ./bin/trash

-include $(DEPS)


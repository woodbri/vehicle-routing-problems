
CPP = g++
UTILP = basePickDeliver
UTIL = baseClasses
BUILD_DIR = ./build
BIN_DIR = ./bin 
INCLUDES = -I../../$(UTIL) -I../$(UTILP) -I.
CPPFLAGS = -g -O0 -MMD  -MP  
LDFLAGS = -lgd $(INCLUDES) -L$(UTIL) -L$(UTILP)

SRCS = init_pd.cpp testconstruction.cpp dumbsolution.cpp pickDelivery.cpp
OBJS = $(addprefix $(BUILD_DIR)/,$(SRCS:.cpp=.o))
HEADS = $(SRCS:.cpp=.h)

UTILP_SRCS = $(wildcard ../$(UTILP)/*.cpp)
UTILP_HDS = $(UTILP_SRCS:.cpp=.h)
UTILP_OBJS = $(addprefix $(BUILD_DIR)/$(UTILP)/,$(notdir $(UTILP_SRCS:.cpp=.o)))
UTILP_HEADS = $(UTILP_HDS)

UTIL_SRCS = $(wildcard ../../$(UTIL)/*.cpp)
UTIL_HDS = $(UTIL_SRCS:.cpp=.h)
UTIL_OBJS = $(addprefix $(BUILD_DIR)/$(UTIL)/,$(notdir $(UTIL_SRCS:.cpp=.o)))
UTIL_HEADS = ../../$(UTIL)/twpath.h ../../$(UTIL)/twc.h ../../$(UTIL)/twbucket.h ../../$(UTIL)/plot.h


DEPS = $(addprefix $(BUILD_DIR)/,$(SRCS:.cpp=.d))




all: $(OBJS) $(UTIL_OBJS) $(UTILP_OBJS) build/pickDelivery.o ./bin/pickDelivery 


bin/pickDelivery: $(OBJS) $(UTIL_OBJS) $(UTILP_OBJS) $(UTILP_HEADS) $(UTIL_HEADS) 
	$(CPP) $^ -o $@ $(LDFLAGS)


build/basePickDeliver/%.o: ../$(UTILP)/%.cpp ../$(UTILP)/%.h $(UTIL_HEADS) $(UTILP_HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

build/baseClasses/%.o: ../../$(UTIL)/%.cpp ../../$(UTIL)/%.h 
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_DIR)/%.o: %.cpp %.h  
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@


$(BUILD_DIR)/pickDelivery.o: pickDelivery.cpp 
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

test: ./bin/pickDelivery ./data/lc008.txt
	./bin/pickDelivery ./data/lc008.txt >./data/small.out

valgrind: ./bin/pickDelivery ./data/lc008.txt
	valgrind -v --track-origins=yes --leak-check=full ./bin/pickDelivery data/lc008.txt

callgrind-8: ./bin/pickDelivery ./data/lc008.txt
	valgrind --tool=callgrind --callgrind-out-file=callgrind-008.out ./bin/pickDelivery data/lc008.txt
	callgrind_annotate callgrind-8.out | tee callgrind-8.txt


.PHONY: clean

clean:
	rm -f tester $(OBJS) $(UTIL_OBJS) $(UTILP_OBJS)  
	find . -name "*.d" -type f -delete
	find . -name "*.png" -type f -delete


cleanAll: clean
	$(MAKE) -C ../$(UTILP) clean
	$(MAKE) -C ../../$(UTIL) clean
	$(MAKE) -C ../../$(UTIL)/tests clean
	$(MAKE) -C ../../trash-collection clean
	rm ./bin/pickDelivery
        
-include $(DEPS)


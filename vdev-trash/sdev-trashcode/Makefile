
CPP = g++
UTILP = baseTrash
UTIL = baseClasses
BUILD_DIR = ./build
BIN_DIR = ./bin 
INCLUDES = -I../../$(UTIL) -I../../$(UTILP) -I.
CPPFLAGS = -g -O0 -MMD  -MP  
LDFLAGS = -lgd $(INCLUDES) -L$(UTIL) -L$(UTILP)

SRCS = trashproblem.cpp trash.cpp
OBJS = $(addprefix $(BUILD_DIR)/,$(SRCS:.cpp=.o))
HEADS = $(SRCS:.cpp=.h)

UTILP_SRCS = $(wildcard ../../$(UTILP)/*.cpp)
UTILP_HDS = $(UTILP_SRCS:.cpp=.h)
UTILP_OBJS = $(addprefix $(BUILD_DIR)/$(UTILP)/,$(notdir $(UTILP_SRCS:.cpp=.o)))
UTILP_HEADS = $(UTILP_HDS)

UTIL_SRCS = $(wildcard ../../$(UTIL)/*.cpp)
UTIL_HDS = $(UTIL_SRCS:.cpp=.h)
UTIL_OBJS = $(addprefix $(BUILD_DIR)/$(UTIL)/,$(notdir $(UTIL_SRCS:.cpp=.o)))
UTIL_HEADS = ../../$(UTIL)/twpath.h ../../$(UTIL)/twc.h ../../$(UTIL)/twbucket.h ../../$(UTIL)/plot.h


DEPS = $(addprefix $(BUILD_DIR)/,$(SRCS:.cpp=.d))




all: $(OBJS) $(UTIL_OBJS) $(UTILP_OBJS) build/trash.o ./bin/trash 


bin/trash: $(OBJS) $(UTIL_OBJS) $(UTILP_OBJS) $(UTILP_HEADS) $(UTIL_HEADS) 
	$(CPP) $^ -o $@ $(LDFLAGS)


build/baseTrash/%.o: ../../$(UTILP)/%.cpp ../../$(UTILP)/%.h $(UTIL_HEADS) $(UTILP_HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

build/baseClasses/%.o: ../../$(UTIL)/%.cpp ../../$(UTIL)/%.h 
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_DIR)/%.o: %.cpp %.h  
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@


$(BUILD_DIR)/trash.o: trash.cpp 
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

test: ./bin/trash ./montevideo/muni-b.containers.txt ./montevideo/muni-b.depots.txt ./montevideo/muni-b.dumps.txt ./montevideo/muni-b.dmatrix-time.txt ./montevideo/muni-b.vehicles.txt
	./bin/trash ./montevideo/muni-b >./montevideo/muni-b.out

valgrind: ./bin/trash ./data/p10a.txt
	valgrind -v --track-origins=yes --leak-check=full ./bin/trash montevideo/muni-b

callgrind-8: ./bin/trash ./data/p10.txt
	valgrind --tool=callgrind --callgrind-out-file=callgrind-008.out ./bin/trash montevideo/muni-b
	callgrind_annotate callgrind-8.out | tee callgrind-8.txt


.PHONY: clean

clean:
	rm -f tester $(OBJS) $(UTIL_OBJS) $(UTILP_OBJS)  
	find . -name "*.d" -type f -delete
	find . -name "*.png" -type f -delete
	mkdir -p $(BIN_DIR)
	mkdir -p $(BUILD_DIR)/$(UTIL) $(BUILD_DIR)/$(UTILP)


cleanAll: clean
	$(MAKE) -C ../$(UTILP) clean
	$(MAKE) -C ../../$(UTIL) clean
	$(MAKE) -C ../../$(UTIL)/tests clean
	$(MAKE) -C ../../trash-collection clean
	$(MAKE) -C ../../pick-deliver clean
	rm ./bin/trash
        
-include $(DEPS)


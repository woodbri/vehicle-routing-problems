
CPP = g++


BASETRASH = baseTrash
BASECLASSES = baseClasses
INITTRASH = initTrash

BUILD_DIR = ./build
BIN_DIR = ./bin

INCLUDES = -I../../$(BASECLASSES) -I../../$(BASETRASH) -I../../$(INITTRASH) -I.
CPPFLAGS = -g -O0 -MMD  -MP
LDFLAGS = -lgd $(INCLUDES) -L$(BASECLASSES) -L$(BASETRASH) -L$(INITTRASH)

SRCS =  dumbsolution.cpp
OBJS = $(addprefix $(BUILD_DIR)/,$(SRCS:.cpp=.o))
HEADS = $(SRCS:.cpp=.h)



BASECLASSES_SRCS = $(wildcard ../../$(BASECLASSES)/*.cpp)
BASECLASSES_HDS = $(UTIL_SRCS:.cpp=.h)
BASECLASSES_OBJS = $(addprefix $(BUILD_DIR)/$(BASECLASSES)/,$(notdir $(BASECLASSES_SRCS:.cpp=.o)))
BASECLASSES_HEADS = ../../$(BASECLASSES)/twpath.h ../../$(BASECLASSES)/twc.h ../../$(BASECLASSES)/twbucket.h ../../$(BASECLASSES)/plot.h $(BASECLASSES_HDS)


BASETRASH_SRCS = $(wildcard ../../$(BASETRASH)/*.cpp)
BASETRASH_HDS = $(BASETRASH_SRCS:.cpp=.h)
BASETRASH_OBJS = $(addprefix $(BUILD_DIR)/$(BASETRASH)/,$(notdir $(BASETRASH_SRCS:.cpp=.o)))
BASETRASH_HEADS = $(BASETRASH_HDS) $(BASECLASSES_HEADS)

INITTRASH_SRCS = $(wildcard ../../$(INITTRASH)/*.cpp)
INITTRASH_HDS = $(INITTRASH_SRCS:.cpp=.h)
INITTRASH_OBJS = $(addprefix $(BUILD_DIR)/$(INITTRASH)/,$(notdir $(INITTRASH_SRCS:.cpp=.o)))
INITTRASH_HEADS = $(INITTRASH_HDS)  $(BASECLASSES_HEADS)



DEPS = $(addprefix $(BUILD_DIR)/,$(SRCS:.cpp=.d))




all:   $(OBJS) $(BASECLASSES_OBJS) $(BASETRASH_OBJS) $(INITTRASH_OBJS) build/trash.o ./bin/trash



build/trash.o: trash.cpp $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS) $(HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

build/baseClasses/%.o: ../../$(BASECLASSES)/%.cpp  ../../$(BASECLASSES)/%.h $(BASECLASSES_HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

build/baseTrash/%.o: ../../$(BASETRASH)/%.cpp ../../$(BASETRASH)/%.h  $(BASECLASSES_HEADS) $(BASETRASH_HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@


build/initTrash/%.o: ../../$(INITTRASH)/%.cpp  ../../$(INITTRASH)/%.h  $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

build/%.o: %.cpp %.h $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS)
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

bin/trash: build/trash.o $(OBJS) $(BASECLASSES_OBJS) $(BASETRASH_OBJS) $(INITTRASH_OBJS) $(BASECLASSES_HEADS) $(BASETRASH_HEADS) $(INITTRASH_HEADS)
	$(CPP) $^ -o $@ $(LDFLAGS)

test: ./bin/trash ./montevideo/muni-b.containers.txt ./montevideo/muni-b.otherlocs.txt ./montevideo/muni-b.dmatrix-time.txt ./montevideo/muni-b.vehicles.txt
	./bin/trash ./montevideo/muni-b >./montevideo/muni-b.out

valgrind: ./bin/trash ./data/p10a.txt
	valgrind -v --track-origins=yes --leak-check=full ./bin/trash montevideo/muni-b

callgrind-8: ./bin/trash ./data/p10.txt
	valgrind --tool=callgrind --callgrind-out-file=callgrind-008.out ./bin/trash montevideo/muni-b
	callgrind_annotate callgrind-8.out | tee callgrind-8.txt


.PHONY: clean

clean:
	rm -f trash $(OBJS) $(BASECLASSES_OBJS) $(BASETRASH_OBJS) $(INITTRASH_OBJS)
	find . -name "*.d" -type f -delete
	find . -name "*.png" -type f -delete
	mkdir -p $(BIN_DIR)
	echo $(INITTRASH)
	mkdir -p $(BUILD_DIR)/$(BASECLASSES) $(BUILD_DIR)/$(BASETRASH) $(BUILD_DIR)/$(INITTRASH)


cleanAll: clean
	$(MAKE) -C ../../$(BASECLASSES) clean
	$(MAKE) -C ../../$(BASETRASH) clean
	$(MAKE) -C ../../$(INITTRASH) clean
	$(MAKE) -C ../../trash-collection clean
	$(MAKE) -C ../../pick-deliver clean
	rm ./bin/trash

-include $(DEPS)


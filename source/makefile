WORKSPACE_ROOT=/home/prash/workspace
CERES_ROOT=$(WORKSPACE_ROOT)/libs/ceres-solver

CC = g++
CFLAGS = -Wall

INCLUDE_DIRS=-I$(CERES_ROOT)/include \
	-I/usr/include/eigen3

LIB_DIRS= \
	-L$(CERES_ROOT)/ceres-bin/lib \
	-L/usr/lib/x86_64-linux-gnu

LIBS = -lceres \
	-lgtest \
	-lutil \
	-lglog \
	-lpthread \
	-lgomp \
	-lcholmod \
	-lblas \
	-llapack

TREE_DIRS= \
	./bin

OUT_DIR = ./bin

SRCS = \
	estimate-camera-params.cpp

OBJS = $(patsubst %.cpp,$(OUT_DIR)/%.o,$(SRCS))

TARGET=$(OUT_DIR)/estimate-camera-params

$(OUT_DIR)/%.o:%.cpp
	$(CC) $(CFLAGS) $(INCLUDE_DIRS) -c $<  -o $@

$(TARGET):$(OBJS)
	$(CC) $(CFLAGS) $(INCLUDE_DIRS) -o $@ $< $(LIB_DIRS) $(LIBS)

tree:
	mkdir -p $(TREE_DIRS)

all: tree $(TARGET)

clean:
	rm -rf ./bin
	
print:
	$(info $(LIB_DIRS))


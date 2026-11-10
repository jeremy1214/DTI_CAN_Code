ifeq ($(OS),Windows_NT)
    SHELL := cmd.exe
    MKDIR = if not exist bin mkdir bin
    BIN_TEST = bin\dti_test.exe
    BIN_DEMO = bin\dti_demo.exe
    RUN_TEST = bin\dti_test.exe
    RUN_DEMO = bin\dti_demo.exe
    CLEAN_CMD = if exist src\*.o del /Q /F src\*.o && if exist bin\*.exe del /Q /F bin\*.exe
else
    SHELL := /bin/sh
    MKDIR = mkdir -p bin
    BIN_TEST = bin/dti_test
    BIN_DEMO = bin/dti_demo
    RUN_TEST = ./bin/dti_test
    RUN_DEMO = ./bin/dti_demo
    CLEAN_CMD = rm -f src/*.o bin/*.exe
endif

CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2 -Iinclude

SRCS = src/dti_codec.cpp src/dti_receiver.cpp src/dti_logger.cpp
OBJS = $(SRCS:.cpp=.o)

all: $(BIN_TEST) $(BIN_DEMO)

bin:
	$(MKDIR)

src/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BIN_TEST): bin $(OBJS) examples/test_codec.cpp
	$(CXX) $(CXXFLAGS) $(OBJS) examples/test_codec.cpp -o $(BIN_TEST)

$(BIN_DEMO): bin $(OBJS) examples/demo_integration.cpp
	$(CXX) $(CXXFLAGS) $(OBJS) examples/demo_integration.cpp -o $(BIN_DEMO)

test: $(BIN_TEST)
	$(RUN_TEST)

demo: $(BIN_DEMO)
	$(RUN_DEMO)

clean:
	$(CLEAN_CMD)

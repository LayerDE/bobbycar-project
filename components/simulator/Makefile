CXX = g++ -Wall -pedantic -std=c++11
MAIN_BINARIES = $(basename $(wildcard *Main.cpp))
HEADERS = $(wildcard *.hpp)
OBJECTS = $(addsuffix .o, $(basename $(filter-out %Main.cpp, $(wildcard *.cpp))))
LIBRARIES = -lm

.PRECIOUS: %.o
.SUFFIXES:
.PHONY: all compile

all: compile

compile: $(MAIN_BINARIES)

clean:
	rm -f *.o
	rm -f $(MAIN_BINARIES)
	rm -f $(TEST_BINARIES)

%Main: %Main.o $(OBJECTS)
	$(CXX) -o $@ $^ $(LIBRARIES)

%.o: %.cpp $(HEADERS)
	$(CXX) -c $<
CXX      := g++
CXXFLAGS := -O2 -std=c++17 -Wall -Wextra -pedantic  -pthread
TARGET   := robotCtl
SRC      := robotCtl.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -f $(TARGET)

run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run

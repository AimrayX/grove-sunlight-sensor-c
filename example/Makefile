# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -I./..

# Source files
SRC = main.c ../si1151.c

# Output executable name
TARGET = si1151_example

# Libraries to link (i2c-dev for I2C)
LDLIBS = -li2c

# Default target
all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

# Clean build artifacts
clean:
	rm -f $(TARGET)
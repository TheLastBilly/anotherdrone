PROJECT_NAME	:= pdb
PORT			?= /dev/ttyUSB0
DEVICE			?= attiny814
PROGRAMMER		:= jtag2udpi
CLOCK			:= 20000000
BAUDRATE		:= 115200
TOOLCHAIN		:= $(HOME)/sdk/avr8-gnu-toolchain-linux_x86_64
CC				:= $(TOOLCHAIN)/bin/avr-gcc
LD				:= $(TOOLCHAIN)/bin/avr-ld
OBJCOPY			:= $(TOOLCHAIN)/bin/avr-objcopy
SIZE			:= $(TOOLCHAIN)/bin/avr-size
CFLAGS   		:= -Wall -Os -std=gnu99 -mmcu=$(DEVICE) -DF_CPU=$(CLOCK) -I. -ffunction-sections -fshort-enums
LDFLAGS			:= -g -mmcu=attiny814
INCFLAGS		:= 

SOURCE_DIR		:= .
BUILD_DIR		:= build
SRCS			:= $(shell find $(SOURCE_DIR) -maxdepth 1 -name '*.c')
OBJS			:= $(SRCS:$(SOURCE_DIR)/%.c=$(BUILD_DIR)/%.o)
TARGET			:= $(BUILD_DIR)/$(PROJECT_NAME)
ELF				:= $(TARGET).elf
HEX				:= $(TARGET).hex

all: $(HEX) $(ELF) $(OBJS)

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

$(BUILD_DIR)/%.elf: $(OBJS)
	@mkdir -p $(dir $@)
	$(CC) -o $@ $(OBJS) $(LDFLAGS)

$(BUILD_DIR)/%.o: $(SOURCE_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCFLAGS) -c $< -o $@

upload:
	avrdude -v -p $(DEVICE) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -U flash:w:$(FILENAME).hex:i

clean:
	@rm -rf $(BUILD_DIR)

fuse:
	# Enable 20MHz clock by writing 0x7E on Fuse 2
	# Taken from https://michael-crum.com/attiny1616/
	avrdude -v -p $(DEVICE) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -U fuse2:w:0x7e:m

.PHONY: all clean fuse upload 


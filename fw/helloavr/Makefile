MCU=atmega32u4
CFLAGS=-Wall -mmcu=$(MCU) -Os
LDFLAGS=-Wl,-relax
CC=avr-gcc

TARGET=hello

.PHONY: all clean install launch

all: $(TARGET).hex
$(TARGET).hex: $(TARGET)

clean:
	rm -f $(TARGET) *.o *.hex

install: $(TARGET).hex
	dfu-programmer $(MCU) erase
	dfu-programmer $(MCU) flash $<
	dfu-programmer $(MCU) launch

launch:
	dfu-programmer $(MCU) launch

%.hex: %
	avr-objcopy -O ihex $< $@

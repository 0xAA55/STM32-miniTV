CROSS_COMPILE=arm-none-eabi-
CC=$(CROSS_COMPILE)gcc
LD=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
CFLAGS=-mcpu=cortex-m7 -std=gnu11 -ffunction-sections -fdata-sections -Wall --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -O3 -flto=auto
LDFLAGS=-mcpu=cortex-m7 -T"FLASHROM.ld" --specs=nosys.specs -Wl,--gc-sections -static -flto=auto --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -Wl,--end-group
LDLIBS=-lc -lm

OBJS=entry.o battery.o folder.o usbmsc.o font_chars.o font12px.o font14px.o font17px.o

All: FlashROM.bin

FlashROM.bin: FlashROM.elf
	$(OBJCOPY) -O binary $^ $@

FlashROM.elf: FLASHROM.ld $(OBJS)
	$(LD) $(LDFLAGS) $(OBJS) -o $@ $(LDLIBS)

clean:
	rm -f $(OBJS) *.bin *.elf

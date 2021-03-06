export

DEVICE_FAMILY = STM32F4xx
DEVICE_TYPE = STM32F429xx
STARTUP_FILE = stm32f429xx
SYSTEM_FILE = stm32f4xx

CMSIS = Drivers/CMSIS
CMSIS_DEVSUP = $(CMSIS)/Device/ST/$(DEVICE_FAMILY)/
CMSIS_OPT = -D$(DEVICE_TYPE) -DUSE_HAL_DRIVER
OTHER_OPT = "-D__weak=__attribute__((weak))" "-D__packed=__attribute__((__packed__))" 
CPU = -mthumb -mcpu=cortex-m4  -mfpu=fpv4-sp-d16 -mfloat-abi=softfp
BINPATH=/usr/local/gcc-arm-none-eabi-5_4-2016q2/bin
SYSTEM = $(BINPATH)/arm-none-eabi

#LDSCRIPT = STM32F407ZG_FLASH.ld
LDSCRIPT = "TrueSTUDIO/com terminal/STM32F429ZI_FLASH.ld"

SRCDIR := Src/
INCDIR := Inc/

LIBDIR := Drivers/


#LIBINC := $(shell find $(INCDIR) -name *.h -printf "-I%h/\n"|sort|uniq)
#LIBINC += $(shell find $(LIBDIR) -name *.h -printf "-I%h/\n"|sort|uniq)
#LIBINC += $(shell find $(MDLDIR) -name *.h -printf "-I%h/\n"|sort|uniq|sed -e 's/lwip\///'|sed -e 's/arch\///')

LIBINC := -IInc
LIBINC += -IDrivers/STM32F4xx_HAL_Driver/Inc
LIBINC += -IDrivers/CMSIS/Include
LIBINC += -IDrivers/CMSIS/Device/ST/STM32F4xx/Include




LIBS := ./$(LIBDIR)/STM32F4xx_HAL_Driver/libstm32fw.a

	   
	   
LIBS += -lm
CC      = $(SYSTEM)-gcc
CCDEP   = $(SYSTEM)-gcc
LD      = $(SYSTEM)-gcc
AR      = $(SYSTEM)-ar
AS      = $(SYSTEM)-gcc
OBJCOPY = $(SYSTEM)-objcopy
OBJDUMP	= $(SYSTEM)-objdump
GDB		= $(SYSTEM)-gdb
SIZE	= $(SYSTEM)-size
OCD	= sudo ~/openocd-git/openocd/src/openocd \
		-s ~/openocd-git/openocd/tcl/ \
		-f interface/stlink-v2.cfg \
		-f target/stm32f4x_stlink.cfg

		
		


##CFLAGS  = -std=gnu99 -g -O2 -Wall -Tstm32_flash.ld
##CFLAGS += -mlittle-endian-mthumb-interwork -nostartfiles
##ifeq ($(FLOAT_TYPE), hard)
##CFLAGS += -fsingle-precision-constant -Wdouble-promotion

#CFLAGS += -mfpu=fpv4-sp-d16 -mfloat-abi=softfp
##else
##CFLAGS += -msoft-float
##endif
  
# INCLUDES = -I$(SRCDIR) $(LIBINC)
INCLUDES = $(LIBINC)
CFLAGS  = $(CPU) $(CMSIS_OPT) $(OTHER_OPT) -Wall -fno-common -fno-strict-aliasing -O2 $(INCLUDES) -g -Wfatal-errors -g -ld
ASFLAGS = $(CFLAGS) -x assembler-with-cpp
LDFLAGS = -Wl,--gc-sections,-Map=$*.map,-cref -T $(LDSCRIPT)   $(CPU)
ARFLAGS = cr
OBJCOPYFLAGS = -Obinary
OBJDUMPFLAGS = -S

STARTUP_OBJ = $(CMSIS_DEVSUP)/Source/Templates/gcc/startup_$(STARTUP_FILE).o
SYSTEM_OBJ = $(CMSIS_DEVSUP)/Source/Templates/system_$(SYSTEM_FILE).o

BIN = main.bin

OBJS = $(sort \
 $(patsubst %.c,%.o,$(wildcard Src/*.c)) \
 $(patsubst %.s,%.o,$(wildcard Src/*.s)) \
 $(STARTUP_OBJ) \
 $(SYSTEM_OBJ))

all: $(BIN)

reset:
	$(OCD) -c init -c "reset run" -c shutdown
	#$(GDB) main.out <reset.gdb

flash: $(BIN)
	$(OCD) -c init -c "reset halt" \
	               -c "flash write_image erase "$(BIN)" 0x08000000" \
			       -c "reset run" \
	               -c shutdown
	
$(BIN): main.out
	$(OBJCOPY) $(OBJCOPYFLAGS) main.out $(BIN)
	$(OBJDUMP) $(OBJDUMPFLAGS) main.out > main.list
	$(SIZE) main.out
	@echo Make finished

main.out: $(LIBS) $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

$(LIBS): libs

libs:
	@$(MAKE) -C $(LIBDIR)
	

libclean: clean
	@$(MAKE) -C $(LIBDIR) clean
	

clean:
	-rm -f $(OBJS)
	-rm -f main.list main.out main.hex main.map main.bin .depend

depend dep: .depend

include .depend

.depend: Src/*.c
	$(CCDEP) $(CFLAGS) -MM $^ | sed -e 's@.*.o:@Src/&@' > .depend 

.c.o:
	@echo cc $<
	@$(CC) $(CFLAGS) -c -o $@ $<

.s.o:
	@echo as $<
	@$(AS) $(ASFLAGS) -c -o $@ $<

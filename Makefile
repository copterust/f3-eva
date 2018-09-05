NAME := f3_eva
target := $(shell grep "target =" .cargo/config | awk '{ print $$3 }' | tr -d '"')
release :=
MODE := $(if $(release),release,debug)
RELEASE_FLAG := $(if $(release),--release,)
usart := 1
log := info
fea :=
FEATURES := "--features=$(log)log,usart$(usart),$(fea)"

TARGET_PATH := ./target/$(target)/$(MODE)
BIN := $(TARGET_PATH)/$(NAME)

UNAME := $(shell uname)
ifeq ($(UNAME), Linux)
TTY := /dev/ttyUSB0
endif
ifeq ($(UNAME), Darwin)
TTY := /dev/tty.wchusbserial1420
endif

$(BIN): build

$(BIN).bin: $(BIN)
	arm-none-eabi-objcopy -S -O binary $(BIN) $(BIN).bin

build:
	cargo -v build $(RELEASE_FLAG) --bin $(NAME) $(FEATURES)

flash: $(BIN).bin
	python2 ./loader/stm32loader.py -b 115200 -p $(TTY) -f F3 -e -w $(BIN).bin

load: flash

boad: build
	bobbin -v load $(RELEASE_FLAG) --bin $(NAME) $(FEATURES)

clean:
	cargo -v clean

.PHONY: build

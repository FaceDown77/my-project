CC = avr-gcc
MCU_NAME = atmega128  # MCU 변경

# F_CPU 설정 (16MHz)
F_CPU = 16000000UL

TARGET = main
PROG_TYPE = stk500
PORT_NAME = /dev/ttyUSB0
BAUD = 115200

RM = rm -rf

# 소스 파일 목록 - 여러 폴더에 있는 소스 파일을 포함
SRC = main.c \
      /home/jwpark/miniProject/rcController/periph/gpio/gpio.c \
      /home/jwpark/miniProject/rcController/periph/uart0/uart0.c \
	  /home/jwpark/miniProject/rcController/periph/uart1/uart1.c \
	  /home/jwpark/miniProject/rcController/app/appMain.c  \
	  /home/jwpark/miniProject/rcController/app/service/control_patrol_car.c \
      /home/jwpark/miniProject/rcController/driver/joystick/joystick.c \
	  /home/jwpark/miniProject/rcController/driver/wifi/esp8266.c \
	  /home/jwpark/miniProject/rcController/periph/adc/adc.c \
	  /home/jwpark/miniProject/rcController/periph/serial/softwareserial.c \
	  /home/jwpark/miniProject/rcController/periph/timer/timer.c \
	  /home/jwpark/miniProject/rcController/app/command/at/atcommands.c \
	  /home/jwpark/miniProject/rcController/test/test.c \
	  /home/jwpark/miniProject/rcController/app/listener/listener.c \
	  /home/jwpark/miniProject/rcController/driver/button/button.c

# 오브젝트 파일 목록 (.c -> .o)
OBJ = $(SRC:.c=.o)

# 기본 목표: HEX 파일 생성
$(TARGET): $(OBJ)
	$(CC) -mmcu=$(MCU_NAME) -DF_CPU=$(F_CPU) -Os -o $(TARGET).elf $(OBJ)
	avr-objcopy -O ihex -R .eeprom $(TARGET).elf $(TARGET).hex
	avrdude -b $(BAUD) -p $(MCU_NAME) -c $(PROG_TYPE) -P $(PORT_NAME) -U flash:w:$(TARGET).hex

# 각 .o 파일 생성 규칙
%.o: %.c
	$(CC) -mmcu=$(MCU_NAME) -DF_CPU=$(F_CPU) -Os -c $< -o $@

# 청소: 생성된 .o, .elf, .hex 파일 제거
clean:
	$(RM) $(OBJ) $(TARGET).elf $(TARGET).hex

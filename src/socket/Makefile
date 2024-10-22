CC = gcc
CFLAGS = -Wall -Wextra -O2
TARGET = main

RM = rm -rf

# 소스 파일 목록 - 여러 폴더에 있는 소스 파일을 포함
SRC = main.c \
      /home/jwpark/miniProject/socket/serial/uart/uart.c \
      /home/jwpark/miniProject/socket/client/client.c

# 오브젝트 파일 목록 (.c -> .o)
OBJ = $(SRC:.c=.o)

# 기본 목표: HEX 파일 생성
$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJ)

# 각 .o 파일 생성 규칙
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# 청소: 생성된 .o 파일 및 실행 파일 제거
clean:
	rm -rf $(OBJ) $(TARGET)

# 실행 목표 (clean 후 실행)
run: $(TARGET)
	./$(TARGET)

.PHONY: clean run

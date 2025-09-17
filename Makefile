# Compiler
CC = gcc

# Флаги компиляции
CFLAGS = -fsanitize=address -g -Wall -O0 -std=gnu99 -I./include

# Флаги линковки (библиотеки)
LDFLAGS = -fsanitize=address -lX11 -lm

# Список исходников
SRC := $(shell find src -name "*.c")

# Список объектных файлов
OBJ := $(SRC:.c=.o)

# Название выходного файла
OUT = test

# Финальная сборка
$(OUT): $(OBJ)
	$(CC) $(OBJ) -o $(OUT) $(LDFLAGS)

# Правило компиляции каждого .c → .o
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@
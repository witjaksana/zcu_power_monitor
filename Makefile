CROSS_COMPILE := aarch64-linux-gnu-

ALLC := $(wildcard *.c)
ALLEXEC := $(patsubst %.c, %, $(ALLC))

IDIR =./include
CC=$(CROSS_COMPILE)gcc
CFLAGS=-I$(IDIR) -g -Wall

LIBS=-lm

.PHONY: clean all

all: $(ALLEXEC)

%: %.c
	$(CC) -o $@ $< $(CFLAGS) $(LIBS)

clean:
	$(RM) *.o $(ALLEXEC)

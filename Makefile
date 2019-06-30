CC=gcc
CFLAGS=-g -Wall
LDFLAGS=

target=v4l2_cam
objs=$(patsubst %.c, %.o, $(wildcard *.c))
install_path=/home/river/apps/bin/

all:$(target)

$(target):$(objs)
	$(CC) $^ -o $@ $(LDFLAGS)

.c.o:
	$(CC) -c $< $(CFLASG)

install:
	cp $(target) $(install_path) -rf

.PHONY:
	clean install

clean:
	rm *.o $(target) -rf


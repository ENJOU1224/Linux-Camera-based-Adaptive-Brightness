# 高性能自动亮度控制 Makefile
CC = gcc
CFLAGS = -O3 -Wall
LIBS = -lm

all: autobright autobright_daemon autobright_fixed_point

# 调试版 (全功能/浮点)
autobright: solution.c
	$(CC) $(CFLAGS) solution.c -o autobright $(LIBS)

# 对比测试版 (整型/带输出)
autobright_fixed_point: autobright_fixed_point.c
	$(CC) $(CFLAGS) autobright_fixed_point.c -o autobright_fixed_point $(LIBS)

# 守护进程版 (无输出/整型优化)
autobright_daemon: autobright_daemon.c
	$(CC) $(CFLAGS) autobright_daemon.c -o autobright_daemon $(LIBS)

# 运行对比测试
test_fixed: autobright_fixed_point
	./autobright_fixed_point

clean:
	rm -f autobright autobright_daemon autobright_fixed_point

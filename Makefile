CCFLAGS = -g -O3 -I /usr/local/include -framework OpenGL
LDFLAGS = -L/usr/local/lib -l glfw3 -lbox2d

SRC = main.cc push.cc
HDR = push.hh

all: push

push: $(SRC) $(HDR)
	g++ $(CCFLAGS) $(SRC) $(LDFLAGS) -o $@

clean:
	rm -f push 
	rm -f *.o

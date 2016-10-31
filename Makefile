# macOS 
CCFLAGS = -std=c++11 -g -O3 -I /usr/local/include -framework OpenGL
LDFLAGS = -L/usr/local/lib -l glfw3 -lbox2d

# Linux
#CCFLAGS = -g -O3 `pkg-config --cflags glfw3 box2d`
#LDFLAGS = `pkg-config --libs glfw3 box2d` -lGL


SRC = main.cc world.cc robot.cc box.cc guiworld.cc
HDR = push.hh

all: push

push: $(SRC) $(HDR)
	g++ $(CCFLAGS) $(SRC) $(LDFLAGS) -o $@

clean:
	rm -f push 
	rm -f *.o

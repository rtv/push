CCFLAGS = -I /usr/local/include -framework OpenGL
LDFLAGS = -L/usr/local/lib -l glfw3 -lbox2d

push: main.cc robot.cc
	g++ $(CCFLAGS) $^ $(LDFLAGS) -o $@

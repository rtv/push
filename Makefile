CCFLAGS = -I /usr/local/include -framework OpenGL -g
LDFLAGS = -L/usr/local/lib -l glfw3 -lbox2d

push: main.cc robot.cc robot.hh
	g++ $(CCFLAGS) main.cc robot.cc $(LDFLAGS) -o $@

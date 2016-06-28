CCFLAGS = -I /usr/local/include -framework OpenGL
LDFLAGS = -L/usr/local/lib -l glfw3 -lbox2d

st: main.cc
	g++ $(CCFLAGS) $^ $(LDFLAGS) -o $@

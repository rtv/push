#include <Box2D/Box2D.h>
#include <GLFW/glfw3.h>

#include <vector>

class Light {
public:
 float intensity;
  float x, y, z; // location (z is height above the x,y plane of the world)
  
  Light( float x, float y, float z, float intensity ) : 
    x(x),y(y),z(z),intensity(intensity)
  {}
};


class Robot;
class Box;

class World 
{
public:  
  b2World* b2world;
  float width, height;
  size_t steps;
  std::vector<Light*> lights;
  std::vector<Box*> boxes;    
  std::vector<Robot*> robots;

  World( float width, float height ) ;
  
  void AddRobot( Robot* robot );
  void AddBox( Box* box );
  void AddLight( Light* light );
  void AddLightGrid( size_t xcount, size_t ycount, float height, float intensity );

  // set the intensity of the light at @index. If @index is out of
  // range, the call has no effect
  void SetLightIntensity( size_t index, float intensity );

  // return instantaneous light intensity from all sources
  float GetLightIntensityAt( float x, float y );

  // perform one simulation step
  void Step( float timestep );
};

class GuiWorld : public World
{
public:
  static bool paused;
  static bool step;
  static int skip;
  
  GLFWwindow* window;
  int draw_interval;
  
  GuiWorld( float width, float height );
  ~GuiWorld();
  
  virtual void Step( float timestep );
  
  bool RequestShutdown();    
};


class Robot
{
public:

  World& world;
  float size;

  float charge; // joules
  float charge_max; // maximum storage capacity
  float charge_delta; // rate of change of charge

  float input_efficiency; // scale input from light sources
  float output_metabolic; // cost per step of just being alive
  float output_efficiency; // scale output due to motion

  static std::vector<Light> lights;
  
  b2Body *body, *bumper;
  b2PrismaticJoint* joint;
  
  Robot( World& world, 
	 float x, float y, float a,  // pose
	 float size=0.5, 
	 float charge=100.0, 
	 float charge_max=100.0,
	 float input_efficiency=0.1,
	 float output_metabolic=0.01,
	 float output_efficiency=0.1 );  
  
  virtual void Update( float timestep );

protected:
  // get sensor data
  float GetLightIntensity( void );
  bool GetBumperPressed( void );

  // send commands
  void SetSpeed( float x, float y, float a );
};

class Box 
{
public:
  float size;

  typedef enum {
    SHAPE_RECT=0,
    SHAPE_HEX
  } box_shape_t;

  b2Body* body;

  Box( World& world, box_shape_t shape, float size );
};



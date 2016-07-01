#include <Box2D/Box2D.h>

#include <vector>

class Robot;

class Light {
public:
 float intensity; // 0..1
  float radius; // meters square
  float x, y; // location
};

// superclass specifies controller interface
class Ctrl
{
public:
  Ctrl(){};
  virtual void Init( Robot& bot ){};
  virtual void Update( Robot& bot, float timestep ){};
};


class Robot
{
public:

  static float SIZE;
  static std::vector<Light> lights;
 
  b2Body *body, *bumper;
  b2PrismaticJoint* joint;
    
  std::vector<Ctrl*> ctrls;

  Robot( b2World& world, float x, float y, float a );  
  void Update( float timestep );

  // add a controller
  void AddController( Ctrl* ctrl )
  {
    ctrls.push_back( ctrl );
  }

  // Initializes all controllers
  void Init( void );

  // get sensor data
  float GetLightIntensity( void );
  bool GetBumperPressed( void );

  // send commands
  void SetSpeed( float x, float y, float a );
};


class DemoPusher : public Ctrl
{
private:
  typedef enum 
    {
      S_PUSH = 0,
      S_BACKUP,
      S_TURN,
      S_COUNT
    } control_state_t;

  static const float PUSH, BACKUP, TURNMAX;
  static const float SPEEDX, SPEEDA;

  float timeleft;
  control_state_t state;
  float speedx, speeda;

public:
  DemoPusher();
  virtual void Init( Robot& bot );
  virtual void Update( Robot& bot, float timestep );
};



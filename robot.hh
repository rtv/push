#include <Box2D/Box2D.h>

#include <vector>

class Light {
public:
 float intensity; // 0..1
  float radius; // meters square
  float x, y; // location
};

class Robot
{
public:

  typedef enum 
    {
      S_PUSH = 0,
      S_BACKUP,
      S_TURN,
      S_COUNT
    } control_state_t;

  static const float PUSH, BACKUP, TURNMAX;
  static const float SPEEDX, SPEEDA;
  static float SIZE;

  static std::vector<Light> lights;

  //float pushTime, backupTime, turnTime;
  float timeleft;
  control_state_t state;
  float speedx, speeda;
  
  b2Body *body, *bumper;
  b2PrismaticJoint* joint;
  
  Robot( b2World& world, float x, float y, float a );
  
  void Update( float timestep );
};


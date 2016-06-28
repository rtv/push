#include <Box2D/Box2D.h>

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

  static const int PUSH, BACKUP, TURNMAX;
  static const float maxspeedx, maxspeeda;
  static const float size;

  int pushTime, backupTime, turnTime;
  control_state_t state;
  float speedx, speeda;
  b2Body* body;
  
  Robot( b2World& world, float x, float y, float a );
  
  void Update();
};

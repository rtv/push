#include <Box2D/Box2D.h>
#include <GLFW/glfw3.h>

#include <vector>

class Robot;

class Light {
public:
 float intensity; // 0..1
  float radius; // meters square
  float x, y; // location
};

class World 
{
public:
  
  b2World* b2world;
  float width, height;
  
  World( float width, float height ) :
    width(width),
    height(height),
    b2world( new b2World( b2Vec2( 0,0 ))) // gravity 
  {
    b2BodyDef groundBodyDef;
    b2PolygonShape groundBox;
    groundBox.SetAsBox( width/2.0, 0.01f );    
    
    b2Body* groundBody[4];
    for( int i=0; i<4; i++ )
      {
	groundBody[i] = b2world->CreateBody(&groundBodyDef);	
	groundBody[i]->CreateFixture(&groundBox, 0.0f);
      }
    
    groundBody[0]->SetTransform( b2Vec2( width/2,0 ), 0 );    
    groundBody[1]->SetTransform( b2Vec2( width/2,height ), 0 );    
    groundBody[2]->SetTransform( b2Vec2( 0, height/2 ), M_PI/2.0 );    
    groundBody[3]->SetTransform( b2Vec2( width, height/2 ), M_PI/2.0 );    
  }

  void Step( float timestep )
  {
    const int32 velocityIterations = 6;
    const int32 positionIterations = 2;

    // Instruct the world to perform a single step of simulation.
    // It is generally best to keep the time step and iterations fixed.
    b2world->Step( timestep, velocityIterations, positionIterations);	
  }
};

class Robot
{
public:
  
  static float size;
  static std::vector<Light> lights;
  
  b2Body *body, *bumper;
  b2PrismaticJoint* joint;
  
  Robot( World& world, float x, float y, float a );  
  
  virtual void Update( float timestep ) = 0; // pure 

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
  static float size;

  b2Body* body;

  Box( World& world )
    : body(NULL)
  {
    b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox( size/2.0, size/2.0 );
    
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &dynamicBox;
    fixtureDef.density = 5;
    fixtureDef.friction = 1.0;
    fixtureDef.restitution = 0.1;

    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    
    body = world.b2world->CreateBody(&bodyDef);    
    body->SetLinearDamping( 10.0 );
    body->SetAngularDamping( 10.0 );
    body->SetTransform( b2Vec2( world.width * drand48(), 
				world.height * drand48()),
			0 );	    	    
      
    body->CreateFixture(&fixtureDef);
  }
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
  
  virtual void Step( float timestep,
		     const std::vector<Robot*>& robots, 
		     const std::vector<Box*>& bodies );
  
  bool RequestShutdown();
    
};

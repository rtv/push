#include <Box2D/Box2D.h>
#include <GLFW/glfw3.h>

#include <vector>

enum _entityCategory {
  BOXBOUNDARY = 0x1,
  ROBOTBOUNDARY = 0x2,
  ROBOT = 0x4,
  BOX = 0x8
};

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

  b2Body* boxWall[4];
  b2Body* robotWall[4];
    
  World( float width, float height ) :
    width(width),
    height(height),
    b2world( new b2World( b2Vec2( 0,0 ))) // gravity 
  {
    // set interior box container
    b2BodyDef boxWallDef;
    b2PolygonShape groundBox;
    groundBox.SetAsBox( width/3.0, 0.01f );    
    
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &groundBox;    
    
    // prevent collision with puck-retaining strings 
    fixtureDef.filter.categoryBits = BOXBOUNDARY;
    fixtureDef.filter.maskBits = BOX; // contain only boxes
  
    for( int i=0; i<4; i++ )
      {
	boxWall[i] = b2world->CreateBody(&boxWallDef);	
	boxWall[i]->CreateFixture(&fixtureDef );
      }
    
    boxWall[0]->SetTransform( b2Vec2( width/2, height/6.0 ), 0 );    
    boxWall[1]->SetTransform( b2Vec2( width/2, height-height/6.0 ), 0 );    
    boxWall[2]->SetTransform( b2Vec2( width/6.0, height/2 ), M_PI/2.0 );    
    boxWall[3]->SetTransform( b2Vec2( width-width/6.0, height/2 ), M_PI/2.0 );


    // set exterior box container
    b2BodyDef boxWallDef1;
    b2PolygonShape groundBox1;
    groundBox1.SetAsBox( width, 0.01f );    
    
    b2FixtureDef fixtureDef1;
    fixtureDef1.shape = &groundBox1;    
    
    // prevent collision with puck-retaining strings 
    fixtureDef1.filter.categoryBits = ROBOTBOUNDARY;
    fixtureDef1.filter.maskBits = ROBOT | BOX; // contain everthing
  
    for( int i=0; i<4; i++ )
      {
	robotWall[i] = b2world->CreateBody(&boxWallDef1);	
	robotWall[i]->CreateFixture(&fixtureDef1 );
      }
    
    robotWall[0]->SetTransform( b2Vec2( width/2,0 ), 0 );    
    robotWall[1]->SetTransform( b2Vec2( width/2,height ), 0 );    
    robotWall[2]->SetTransform( b2Vec2( 0, height/2 ), M_PI/2.0 );    
    robotWall[3]->SetTransform( b2Vec2( width, height/2 ), M_PI/2.0 ); 
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

  typedef enum {
    SHAPE_RECT=0,
    SHAPE_HEX
  } box_shape_t;

  b2Body* body;

  Box( World& world, box_shape_t shape )
    : body(NULL)
  {
    b2PolygonShape dynamicBox;

    switch( shape )
      {
      case SHAPE_RECT:
	dynamicBox.SetAsBox( size/2.0, size/2.0 );
	break;
      case SHAPE_HEX:
	{
	  b2Vec2 verts[6];

	  for (int i = 0; i < 6; i++) {
	    verts[i].x = size/2.0 * cos(2.0 * M_PI * i / 6.0);
	    verts[i].y = size/2.0 * sin(2.0 * M_PI * i / 6.0);
	  }
	
	dynamicBox.Set(verts,6);
	}
	break;
      default:
	std::cout << "invalid shape number " << shape << std::endl;
	break;
      }

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &dynamicBox;
    fixtureDef.density = 5;
    fixtureDef.friction = 1.0;
    fixtureDef.restitution = 0.1;

    fixtureDef.filter.categoryBits = BOX;
    fixtureDef.filter.maskBits = 0xFFFF; //everything
      // BOX | ROBOT | BOXBOUNDARY | ROBOTBOUNDARY; // everything
     
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    
    body = world.b2world->CreateBody(&bodyDef);    
    body->SetLinearDamping( 10.0 );
    body->SetAngularDamping( 10.0 );
    body->SetTransform( b2Vec2( world.width/2 +
				1.9/3.0 * world.width*(drand48()-0.5), 
				world.height/2 +
				1.9/3.0 * world.height*(drand48()-0.5)),
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

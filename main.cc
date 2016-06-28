#include <GLFW/glfw3.h>
#include <Box2D/Box2D.h>
#include <vector>

//using namespace std;
#include <iostream>

double mousex = 0;
double mousey = 0;

void checkmouse( GLFWwindow* win, double x, double y) 
{
  //std::cout << x << ' ' << y << std::endl;
  mousex = x/10.0;
  mousey = -y/10.0;
}

const size_t ROBOTS = 16;
const size_t BODIES = 32;
int DRAW_SKIP = 1;

const float maxspeedx = 0.5;
const float maxspeeda = M_PI/2.0;

const float worldwidth = 6;
const float worldheight = 6;

const float boxside = 0.33;
const float robotside = 0.3;

double speedx = 0.0; // meters per second
double speeda = 0.0; // degrees per second

const float c_yellow[3] = {1.0, 1.0, 0.0 };
const float c_red[3] = {1.0, 0.0, 0.0 };
const float c_tan[3] = { 0.8, 0.6, 0.5};
const float c_gray[3] = { 0.9, 0.9, 0.9 };

// Prepare for simulation. Typically we use a time step of 1/60 of a
// second (60Hz) and 10 iterations. This provides a high quality simulation
// in most game scenarios.
const float32 timeStep = 1.0 / 10.0;
const int32 velocityIterations = 6;
const int32 positionIterations = 2;

void key_callback( GLFWwindow* window, 
		   int key, int scancode, int action, int mods)
{
  if(action == GLFW_PRESS)
    switch( key )
      {
      case GLFW_KEY_W:
        speedx = maxspeedx;
	break;
      case GLFW_KEY_S:
        speedx = -maxspeedx;
	break;
      case GLFW_KEY_A:
        speeda = maxspeeda;
	break;
      case GLFW_KEY_D:
        speeda = -maxspeeda;
	break;
      case GLFW_KEY_LEFT_BRACKET:
	if( mods & GLFW_MOD_SHIFT )
	  DRAW_SKIP = 0;
	else
	  DRAW_SKIP  = std::max( 0, --DRAW_SKIP );
	break;
      case GLFW_KEY_RIGHT_BRACKET:
	if( mods & GLFW_MOD_SHIFT )
	  DRAW_SKIP = 100;
	else
	  DRAW_SKIP  = ++DRAW_SKIP;
	break;
      default:
	break;
      }

 if( action == GLFW_RELEASE )
    switch( key )
      {
      case GLFW_KEY_W:
        if( speedx > 0 ) speedx = 0.0;
	break;
      case GLFW_KEY_S:
        if( speedx < 0 ) speedx = 0.0;
	break;
      case GLFW_KEY_A:
        if( speeda > 0 ) speeda = 0.0;
	break;
      case GLFW_KEY_D:
        if( speeda < 0 ) speeda = 0.0;
	break;
      default:
	break;
      }
}

class Robot
{
public:

  int pushTime, backupTime, turnTime;
  int state;
  float speedx, speeda;
  b2Body* body;
  
  Robot( b2World& world, float x, float y, float a ) : 
    pushTime( random() % 10 ),
    backupTime( 0 ),
    turnTime( 0),
    state( 0 ),
    speedx( 0 ),
    speeda( 0 ),
    body( NULL )
  {
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    body = world.CreateBody(&bodyDef);
    
    b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox( robotside/2.0, robotside/2.0 );
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &dynamicBox;    
    fixtureDef.density = 10;
    fixtureDef.friction = 2.0;
    body->CreateFixture(&fixtureDef);
    body->SetTransform( b2Vec2( x, y ), a );	
    //bodies[0]->SetLinearDamping( 5.0 );
    //bodies[0]->SetAngularDamping( 10.0 );

    //std::cout << "robot mass "  << bodies[0]->GetMass() << std::endl;
  }
  
  void Update()
  {
    switch( state )
      {
      case 0: // push
	speedx = maxspeedx;
	speeda = 0;	    
	if( --pushTime < 1 )
	  {
	    state = 1;
	    pushTime = 20;
	  }
	break;
	
      case 1: // backup
	speedx = -maxspeedx;
	speeda = 0;	    
	if( --backupTime < 1 )
	  {
	    state = 2;
	    backupTime = 5;
	  }
	break;
	
      case 2: // turn
	speedx = 0;
	speeda = maxspeeda;	    
	if( --turnTime < 1 )
	  {
	    state = 0;
	    turnTime = random() % 20;
	  }
	break;
      }

    body->SetLinearVelocity( body->GetWorldVector(b2Vec2( speedx, 0 )));
    body->SetAngularVelocity( speeda );
  }
};

void DrawBody( b2Body* b, const float color[3] )
{
  for (b2Fixture* f = b->GetFixtureList(); f; f = f->GetNext()) 
    {
      switch( f->GetType() )
	{
	case b2Shape::e_circle:
	  {
	    b2CircleShape* circle = (b2CircleShape*)f->GetShape();
	  }
	  break;
	case b2Shape::e_polygon:
	  {
	    b2PolygonShape* poly = (b2PolygonShape*)f->GetShape();
	    
	    const int count = poly->GetVertexCount();
	    
	    //glColor3f( color[0]*0.8, color[1]*0.8, color[2]*0.8 );
	    glColor3fv( color );
	    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL );
	    glBegin( GL_POLYGON );	
	    
	    for( int i = 0; i < count; i++ )
	      {
		const b2Vec2& v = poly->GetVertex( i );
		
		const b2Vec2 w = b->GetWorldPoint( v );
		
		glVertex2f( w.x, w.y );
	      }
	    glEnd();		  
	    
	    glLineWidth( 2.0 );
	    glColor3f( color[0]/5, color[1]/5, color[2]/5 );
	    //glColor3fv( color );
	    //glColor3f( 0,0,0 );
	    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	    glBegin( GL_POLYGON );	
	    
	    for( int i = 0; i < count; i++ )
	       {
		 const b2Vec2& v = poly->GetVertex( i );		 
	         const b2Vec2 w = b->GetWorldPoint( v );		 
	         glVertex2f( w.x, w.y );
	       }
	    glEnd();		  
	  }
	  break;
	default:
	  break;
	} 
    }
}

void UpdateGui( GLFWwindow* window,
		const std::vector<Robot*>& robots, 
		const std::vector<b2Body*>& bodies ) 
{
  glClearColor( 0.8, 0.8, 0.8, 1.0 ); 
  glClear(GL_COLOR_BUFFER_BIT);	
  
  for( int i=0; i<bodies.size(); i++ )
    DrawBody( bodies[i], c_gray );
  
  for( int i=0; i<robots.size(); i++ )
    DrawBody( robots[i]->body, c_red );

  // draw a nose on the robot
  glColor3f( 1,1,1 );
  glPointSize( 12 );
  glBegin( GL_POINTS );
  for( int i=0; i<robots.size(); i++ )
    {      
      const b2Transform& t = robots[i]->body->GetTransform();
      const float a = t.q.GetAngle();

      glVertex2f( t.p.x + robotside/3.0 * cos(a),
		  t.p.y + robotside/3.0 * sin(a) );		  
    }
  glEnd();
  
  glBegin( GL_LINES );
  glVertex2f( 0,0 );
  glVertex2f( worldwidth,0 );
  glVertex2f( 0,0 );
  glVertex2f( 0,worldheight );
  glEnd();
  
  /* Swap front and back buffers */
  glfwSwapBuffers(window);
  
  /* Poll for and process events */
  glfwPollEvents();
}

int main( int argc, char* argv[] )
{
  srand48( time(NULL) );
  
  GLFWwindow* window;
  
  /* Initialize the library */
  if (!glfwInit())
    return -1;
  
  /* Create a windowed mode window and its OpenGL context */
  window = glfwCreateWindow(800, 800, "S3", NULL, NULL);
  if (!window)
    {
      glfwTerminate();
      return -1;
    }
  
  b2Vec2 gravity(0.0f, 0.0f );
  b2World world(gravity);
  b2BodyDef groundBodyDef;
  b2PolygonShape groundBox;
  groundBox.SetAsBox( worldwidth/2.0, 0.01f );    
  
  b2Body* groundBody[4];
  for( int i=0; i<4; i++ )
    {
      groundBody[i] = world.CreateBody(&groundBodyDef);	
      groundBody[i]->CreateFixture(&groundBox, 0.0f);
    }
  
  groundBody[0]->SetTransform( b2Vec2( worldwidth/2,0 ), 0 );    
  groundBody[1]->SetTransform( b2Vec2( worldwidth/2,worldheight ), 0 );    
  groundBody[2]->SetTransform( b2Vec2( 0, worldheight/2 ), M_PI/2.0 );    
  groundBody[3]->SetTransform( b2Vec2( worldwidth, worldheight/2 ), M_PI/2.0 );    
  std::vector<Robot*> robots;
  for( int i=0; i<ROBOTS; i++ )
    robots.push_back( new Robot( world, 
				 drand48() * worldwidth, 
				 drand48() * worldheight, 
				 -M_PI + drand48() * 2.0 * M_PI ));
  
  
  // Define another box shape for our dynamic body.
  b2PolygonShape dynamicBox;
  dynamicBox.SetAsBox( boxside/2.0, boxside/2.0 );
  
  // Define the dynamic body fixture.
  b2FixtureDef fixtureDef;
  fixtureDef.shape = &dynamicBox;
  fixtureDef.density = 5;
  fixtureDef.friction = 3.0;
  fixtureDef.restitution = 0.1;
  
    std::vector<b2Body*> bodies;    
    for( int i=0; i<BODIES; i++ )
      {
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	b2Body* body = world.CreateBody(&bodyDef);
	
	body->SetLinearDamping( 10.0 );
	body->SetAngularDamping( 10.0 );
	body->SetTransform( b2Vec2( worldwidth * drand48(), 
				    worldheight * drand48()),
			    0 );	    	    
	
	body->CreateFixture(&fixtureDef);
	
	
	bodies.push_back( body );
      }
    
    
    /* Make the window's context current */
    glfwMakeContextCurrent(window);
    
    // scale the drawing to fit the whole world in the window, origin
    // at bottom left
    glScalef( 2.0 / worldwidth, 2.0 / worldheight, 1.0 );
    glTranslatef( -worldwidth/2.0, -worldheight/2.0, 0 );

    // get mouse/pointer events
    //glfwSetCursorPosCallback( window, checkmouse );

    // get key events
    glfwSetKeyCallback (window, key_callback);
    
    int draw_interval = DRAW_SKIP;
    
    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
      {
	for( int i=0; i<ROBOTS; i++ )
	  robots[i]->Update();
	
	if( --draw_interval < 1 )
	  {	    
	    UpdateGui( window, robots, bodies );
	    draw_interval = DRAW_SKIP;
	  }

	// Instruct the world to perform a single step of simulation.
	// It is generally best to keep the time step and iterations fixed.
	world.Step(timeStep, velocityIterations, positionIterations);	
      }
    
    glfwTerminate();
    return 0;
}

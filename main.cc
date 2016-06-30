#include <GLFW/glfw3.h>
#include <Box2D/Box2D.h>

#include <getopt.h>
#include <unistd.h> // for usleep(3)

#include <vector>
#include <iostream>

#include "robot.hh"

double mousex = 0;
double mousey = 0;

void checkmouse( GLFWwindow* win, double x, double y) 
{
  //std::cout << x << ' ' << y << std::endl;
  mousex = x/10.0;
  mousey = -y/10.0;
}

bool gui_paused = true;
bool gui_step = false;

const float WORLDWIDTH = 7;
const float WORLDHEIGHT = 7;

size_t ROBOTS = 8;
size_t BODIES = 32;
int DRAW_SKIP = 1;

const float maxspeedx = 0.5;
const float maxspeeda = M_PI/2.0;

float boxside = 0.32;

const float c_yellow[3] = {1.0, 1.0, 0.0 };
const float c_red[3] = {1.0, 0.0, 0.0 };
const float c_darkred[3] = {0.8, 0.0, 0.0 };
const float c_tan[3] = { 0.8, 0.6, 0.5};
const float c_gray[3] = { 0.9, 0.9, 1.0 };

// Prepare for simulation. Typically we use a time step of 1/60 of a
// second (60Hz) and 10 iterations. This provides a high quality simulation
// in most game scenarios.
const float32 timeStep = 1.0 / 30.0;
const int32 velocityIterations = 6;
const int32 positionIterations = 2;

void key_callback( GLFWwindow* window, 
		   int key, int scancode, int action, int mods)
{
  if(action == GLFW_PRESS)
    switch( key )
      {
      case GLFW_KEY_SPACE:
	gui_paused = !gui_paused;
	break;

      case GLFW_KEY_S:
	gui_step = !gui_step;

	if( ! gui_step )
	  gui_paused = false;

	break;
      // case GLFW_KEY_W:
      //   speedx = maxspeedx;
      // 	break;
      // case GLFW_KEY_S:
      //   speedx = -maxspeedx;
      // 	break;
      // case GLFW_KEY_A:
      //   speeda = maxspeeda;
      // 	break;
      // case GLFW_KEY_D:
      //   speeda = -maxspeeda;
      // 	break;
      case GLFW_KEY_LEFT_BRACKET:
	if( mods & GLFW_MOD_SHIFT )
	  DRAW_SKIP = 0;
	else
	  DRAW_SKIP  = std::max( 0, --DRAW_SKIP );
	break;
      case GLFW_KEY_RIGHT_BRACKET:
	if( mods & GLFW_MOD_SHIFT )
	  DRAW_SKIP = 500;
	else
	  DRAW_SKIP  = ++DRAW_SKIP;
	break;
      default:
	break;
      }

 // if( action == GLFW_RELEASE )
 //    switch( key )
 //      {
 //      case GLFW_KEY_W:
 //        if( speedx > 0 ) speedx = 0.0;
 // 	break;
 //      case GLFW_KEY_S:
 //        if( speedx < 0 ) speedx = 0.0;
 // 	break;
 //      case GLFW_KEY_A:
 //        if( speeda > 0 ) speeda = 0.0;
 // 	break;
 //      case GLFW_KEY_D:
 //        if( speeda < 0 ) speeda = 0.0;
 // 	break;
  //      default:
  //	break;
  //    }
}


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
    {
      DrawBody( robots[i]->body, c_red );
      DrawBody( robots[i]->bumper, c_darkred );
    }

  // draw a nose on the robot
  glColor3f( 1,1,1 );
  glPointSize( 12 );
  glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
  glBegin( GL_TRIANGLES );
  for( int i=0; i<robots.size(); i++ )
    {      
      const b2Transform& t = robots[i]->body->GetTransform();
      const float a = t.q.GetAngle();

      glVertex2f( t.p.x + Robot::SIZE/2.0 * cos(a),
		  t.p.y + Robot::SIZE/2.0 * sin(a) );		  
      glVertex2f( t.p.x + Robot::SIZE/3.0 * cos(a+0.5),
		  t.p.y + Robot::SIZE/3.0 * sin(a+0.5) );		  
      glVertex2f( t.p.x + Robot::SIZE/3.0 * cos(a-0.5),
		  t.p.y + Robot::SIZE/3.0 * sin(a-0.5) );		  
    }
  glEnd();
  
  glBegin( GL_LINES );
  glVertex2f( 0,0 );
  glVertex2f( WORLDWIDTH,0 );
  glVertex2f( 0,0 );
  glVertex2f( 0,WORLDHEIGHT );
  glEnd();
  
  /* Swap front and back buffers */
  glfwSwapBuffers(window);
  
  /* Poll for and process events */
  glfwPollEvents();
}

class MyContactListener : public b2ContactListener {
public:
  void BeginContact(b2Contact* contact) 
  {  /* handle begin event */ 
    // set the push time very small
  }
 

 //void EndContact(b2Contact* contact) { /* handle end event */ }
  //void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) { /* handle pre-solve event */ }
  //  void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse)
  //{ /* handle post-solve event */ } 
};

/* options descriptor */
static struct option longopts[] = {
	{ "robots",  required_argument,   NULL,  'r' },
	{ "boxes",  required_argument,   NULL,  'b' },
	{ "robotsize",  required_argument,   NULL,  'z' },
	{ "boxsize",  required_argument,   NULL,  's' },
	//	{ "help",  optional_argument,   NULL,  'h' },
	{ NULL, 0, NULL, 0 }
};

int main( int argc, char* argv[] )
{
  srand48( time(NULL) );
  
  GLFWwindow* window;
 
  int ch=0, optindex=0;
  //bool usegui = true;
  //bool showclock = false;
  
  while ((ch = getopt_long(argc, argv, "r:b:s:z:", longopts, &optindex)) != -1)
    {
      switch( ch )
	{
	case 0: // long option given
	  printf( "option %s given\n", longopts[optindex].name );

          if (optarg)
            printf (" with arg %s", optarg);
          printf ("\n");
	  break;

	case 'r':
	  ROBOTS = atoi( optarg );
	  break;
	case 'b':
	  BODIES = atoi( optarg );
	  break;
	case 'z':
	  Robot::SIZE = atof( optarg );
	  break;
	case 's':
	  boxside = atof( optarg );
	  break;
	// case 'h':  
	// case '?':  
	//   puts( USAGE );
	//   exit(0);
	//   break;
	// default:
	//   printf("unhandled option %c\n", ch );
	//   puts( USAGE );
	//   exit(0);
	}
    }
  
  puts("");// end the first start-up line

 
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
  groundBox.SetAsBox( WORLDWIDTH/2.0, 0.01f );    
  
  b2Body* groundBody[4];
  for( int i=0; i<4; i++ )
    {
      groundBody[i] = world.CreateBody(&groundBodyDef);	
      groundBody[i]->CreateFixture(&groundBox, 0.0f);
    }
  
  groundBody[0]->SetTransform( b2Vec2( WORLDWIDTH/2,0 ), 0 );    
  groundBody[1]->SetTransform( b2Vec2( WORLDWIDTH/2,WORLDHEIGHT ), 0 );    
  groundBody[2]->SetTransform( b2Vec2( 0, WORLDHEIGHT/2 ), M_PI/2.0 );    
  groundBody[3]->SetTransform( b2Vec2( WORLDWIDTH, WORLDHEIGHT/2 ), M_PI/2.0 );    
  std::vector<Robot*> robots;
  for( int i=0; i<ROBOTS; i++ )
    robots.push_back( new Robot( world, 
				 drand48() * WORLDWIDTH, 
				 drand48() * WORLDHEIGHT, 
				 -M_PI + drand48() * 2.0 * M_PI ));
  
  
  // Define another box shape for our dynamic body.
  b2PolygonShape dynamicBox;
  dynamicBox.SetAsBox( boxside/2.0, boxside/2.0 );
  
  // Define the dynamic body fixture.
  b2FixtureDef fixtureDef;
  fixtureDef.shape = &dynamicBox;
  fixtureDef.density = 5;
  fixtureDef.friction = 1.0;
  fixtureDef.restitution = 0.1;
  
    std::vector<b2Body*> bodies;    
    for( int i=0; i<BODIES; i++ )
      {
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	b2Body* body = world.CreateBody(&bodyDef);
	
	body->SetLinearDamping( 10.0 );
	body->SetAngularDamping( 10.0 );
	body->SetTransform( b2Vec2( WORLDWIDTH * drand48(), 
				    WORLDHEIGHT * drand48()),
			    0 );	    	    
	
	body->CreateFixture(&fixtureDef);
	
	
	bodies.push_back( body );
      }
    
    
    /* Make the window's context current */
    glfwMakeContextCurrent(window);
    
    // scale the drawing to fit the whole world in the window, origin
    // at bottom left
    glScalef( 2.0 / WORLDWIDTH, 2.0 / WORLDHEIGHT, 1.0 );
    glTranslatef( -WORLDWIDTH/2.0, -WORLDHEIGHT/2.0, 0 );

    // get mouse/pointer events
    //glfwSetCursorPosCallback( window, checkmouse );

    // get key events
    glfwSetKeyCallback (window, key_callback);
    
    int draw_interval = DRAW_SKIP;
    
    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
      {
	for( int i=0; i<ROBOTS; i++ )
	  robots[i]->Update( timeStep );
	
	if( --draw_interval < 1 )
	  {	    
	    UpdateGui( window, robots, bodies );
	    draw_interval = DRAW_SKIP;
	  }

	if( ! gui_paused )
	  {
	    // Instruct the world to perform a single step of simulation.
	    // It is generally best to keep the time step and iterations fixed.
	    world.Step(timeStep, velocityIterations, positionIterations);	

	    if( gui_step )
	      gui_paused = true;


	    // b2Vec2 force = robots[0]->joint->GetReactionForce( 1.0/timeStep );
	    // float trans = robots[0]->joint->GetJointTranslation();
	    
	    // std::cout << "bump force " << force.x << ' ' << force.y << " trans " << trans << std::endl;
	    
	  }
	else
	  usleep(1000);
      }
    
    glfwTerminate();
    return 0;
}

#include <iostream>

#include "push.hh"

std::vector<Light> Robot::lights(2);

float Box::size = 0.32;
float Robot::size = 0.3;

// constructor
  Robot::Robot( World& world, const float x, const float y, const float a ) : 
  body( NULL ),
  joint( NULL )
{
  lights[0].x = 3;
  lights[0].y = 4;
  lights[0].intensity = 0.6;

  lights[1].x = 6;
  lights[1].y = 5;
  lights[1].intensity = 0.8;

  // lights[2].x = 4;
  // lights[2].y = 2;
  // lights[2].intensity = 0.6;

  // lights[3].x = 5;
  // lights[3].y = 2;
  // lights[3].intensity = 0.6;

  // lights[4].x = 5;
  // lights[4].y = 5;
  // lights[4].intensity = 0.6;

  // lights[5].x = 6;
  // lights[5].y = 6;
  // lights[5].intensity = 0.6;

  b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;
  body = world.b2world->CreateBody(&bodyDef);
  bumper = world.b2world->CreateBody(&bodyDef);
  
  b2PolygonShape dynamicBox;
  dynamicBox.SetAsBox( size/2.0, size/2.0 );
  
  b2FixtureDef fixtureDef;
  fixtureDef.shape = &dynamicBox;    
  fixtureDef.density = 10;
  fixtureDef.friction = 1.0;
  body->CreateFixture(&fixtureDef);
  
  // bumper has same settings the body but different size
  dynamicBox.SetAsBox( size/10.0, size/2.0 );
  bumper->CreateFixture(&fixtureDef);

  
  b2PrismaticJointDef jointDef;
  
  jointDef.Initialize( body, 
		       bumper, 
		       body->GetWorldCenter(), 
		       b2Vec2( 1.0f, 0.0f )
		       ); 
  
  jointDef.lowerTranslation = 0;//-0.2;
  jointDef.upperTranslation = 0.04f;
  jointDef.enableLimit = true;
  jointDef.maxMotorForce = 0.8f;
  jointDef.motorSpeed = 1.0f;
  jointDef.localAnchorA.Set( size/2.0, 0); // on the nose
  jointDef.localAnchorB.Set( 0,0 );
  
  jointDef.enableMotor = true;
  //jointDef.collideConnected = true;
  
  joint = (b2PrismaticJoint*)world.b2world->CreateJoint( &jointDef );

  // place assembled robot in the world
  body->SetTransform( b2Vec2( x, y ), a );	
  bumper->SetTransform( body->GetWorldPoint( b2Vec2( size/2,0) ), a );	
}

float Robot::GetLightIntensity( void )
{
  b2Vec2 here = body->GetWorldCenter();
  
  // integrate brightness over all light sources
  float brightness = 0.0;
  for( std::vector<Light>::iterator it = Robot::lights.begin(); 
       it != Robot::lights.end(); 
       it++ )
    {
      const float distanceToLightSqrd = 
	pow( here.x - it->x, 2.0 ) + 
	pow( here.y - it->y, 2.0 );
      
      brightness += it->intensity / distanceToLightSqrd;
    }

  return brightness;
}

bool Robot::GetBumperPressed( void )
{
  return( joint->GetJointTranslation() < 0.01 );
}

// set body speed in body-local coordinate frame
void Robot::SetSpeed( float x, float y, float a )
{  
  body->SetLinearVelocity( body->GetWorldVector(b2Vec2( x, y )));
  body->SetAngularVelocity( a );
}


const float c_yellow[3] = {1.0, 1.0, 0.0 };
const float c_red[3] = {1.0, 0.0, 0.0 };
const float c_darkred[3] = {0.8, 0.0, 0.0 };
const float c_tan[3] = { 0.8, 0.6, 0.5};
const float c_gray[3] = { 0.9, 0.9, 1.0 };

bool GuiWorld::paused = true;
bool GuiWorld::step = false;
int GuiWorld::skip = 1;


// void checkmouse( GLFWwindow* win, double x, double y) 
// {
//   //std::cout << x << ' ' << y << std::endl;
//   mousex = x/10.0;
//   mousey = -y/10.0;
// }

void key_callback( GLFWwindow* window, 
		   int key, int scancode, int action, int mods)
{
  if(action == GLFW_PRESS)
    switch( key )
      {
      case GLFW_KEY_SPACE:
	GuiWorld::paused = !GuiWorld::paused;
	break;

      case GLFW_KEY_S:	
	GuiWorld::paused = true;
	GuiWorld::step = true; //!GuiWorld::step;
	break;

      case GLFW_KEY_LEFT_BRACKET:
	if( mods & GLFW_MOD_SHIFT )
	  GuiWorld::skip = 0;
	else
	  GuiWorld::skip  = std::max( 0, --GuiWorld::skip );
	break;
      case GLFW_KEY_RIGHT_BRACKET:
	if( mods & GLFW_MOD_SHIFT )
	  GuiWorld::skip = 500;
	else
	  GuiWorld::skip++;
	break;
      default:
	break;
      }
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
	    
	    glColor3fv( color );
	    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL );
	    glBegin( GL_POLYGON );	
	    
	    for( int i = 0; i < count; i++ )
	      {
		const b2Vec2 w = b->GetWorldPoint( poly->GetVertex( i ));		
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


void DrawDisk(float cx, float cy, float r ) 
{ 
  const int num_segments = 32.0 * sqrtf( r );
  
  const float theta = 2 * M_PI / float(num_segments); 
  const float c = cosf(theta);//precalculate the sine and cosine
  const float s = sinf(theta);
  float t;
  
  float x = r; //we start at angle = 0 
  float y = 0; 
  
  //glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
  glBegin(GL_TRIANGLE_STRIP); 
  for(int ii = 0; ii < num_segments; ii++) 
    { 
      glVertex2f( x + cx, y + cy);//output vertex 
      glVertex2f( cx, cy );//output vertex 
      
      //apply the rotation matrix
      t = x;
      x = c * x - s * y;
      y = s * t + c * y;
    } 

  glVertex2f( r + cx, 0 + cy); // first point again to close disk

  glEnd(); 
}

GuiWorld::GuiWorld( float width, float height ) :
  World( width, height ),
  window(NULL),
  draw_interval( skip )
  {
    srand48( time(NULL) );  

    /* Initialize the gui library */
    if (!glfwInit())
      {
	std::cout << "Failed glfwInit()" << std::endl;
	exit(1);
      }
  
    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(800, 800, "S3", NULL, NULL);
    if (!window)
      {
	glfwTerminate();
	exit(2);
      }
  
    /* Make the window's context current */
    glfwMakeContextCurrent(window);
    
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // scale the drawing to fit the whole world in the window, origin
    // at bottom left
    glScalef( 2.0 / width, 2.0 / height, 1.0 );
    glTranslatef( -width/2.0, -height/2.0, 0 );
    
    // get mouse/pointer events
    //glfwSetCursorPosCallback( window, checkmouse );
    
    // get key events
    glfwSetKeyCallback (window, key_callback);
}

void GuiWorld::Step( float timestep, 
		const std::vector<Robot*>& robots, 
		const std::vector<Box*>& bodies ) 
{
  if( !paused || step)
    {
      World::Step( timestep );

      step = false;
      //	paused = true;
    }

  if( --draw_interval < 1 )
    {
      draw_interval = skip;

      glClearColor( 0.8, 0.8, 0.8, 1.0 ); 
      glClear(GL_COLOR_BUFFER_BIT);	
      
      for( int i=0; i<bodies.size(); i++ )
	DrawBody( bodies[i]->body, c_gray );
      
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
	  
	  glVertex2f( t.p.x + Robot::size/2.0 * cos(a),
		      t.p.y + Robot::size/2.0 * sin(a) );		  
	  glVertex2f( t.p.x + Robot::size/3.0 * cos(a+0.5),
		      t.p.y + Robot::size/3.0 * sin(a+0.5) );		  
	  glVertex2f( t.p.x + Robot::size/3.0 * cos(a-0.5),
		      t.p.y + Robot::size/3.0 * sin(a-0.5) );		  
	}
      glEnd();
      
      glBegin( GL_LINES );
      glVertex2f( 0,0 );
      glVertex2f( width,0 );
      glVertex2f( 0,0 );
      glVertex2f( 0,height );
      glEnd();
      
      // draw the light sources  
      glColor4f( 1,1,0,0.2 );
      for( std::vector<Light>::iterator it = Robot::lights.begin(); 
	   it != Robot::lights.end(); 
	   it++ )
	{
	  DrawDisk( it->x, it->y, sqrt( it->intensity ) );
	}
      
      /* Swap front and back buffers */
      glfwSwapBuffers(window);
      
      /* Poll for and process events */
      glfwPollEvents();
    }
}

GuiWorld::~GuiWorld( void )
{
  glfwTerminate();
}

bool GuiWorld::RequestShutdown( void )
{
  return glfwWindowShouldClose(window);
}

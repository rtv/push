#include <iostream>

#include "push.hh"

const double c_yellow[3] = {1.0, 1.0, 0.0 };
const double c_red[3] = {1.0, 0.0, 0.0 };
const double c_darkred[3] = {0.8, 0.0, 0.0 };
const double c_tan[3] = { 0.8, 0.6, 0.5};
const double c_gray[3] = { 0.9, 0.9, 1.0 };

bool GuiWorld::paused = false;
bool GuiWorld::step = false;
int GuiWorld::skip = 10;

void DrawDisk(double cx, double cy, double r );


double RTOD( double rad )
{
  return rad * 180/M_PI;
}

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



void DrawBody( b2Body* b, const double color[3] )
{
  for (b2Fixture* f = b->GetFixtureList(); f; f = f->GetNext()) 
    {
      switch( f->GetType() )
	{
	case b2Shape::e_circle:
	  {
	    b2CircleShape* circle = (b2CircleShape*)f->GetShape();

	    b2Vec2 pos = b->GetPosition();
	    DrawDisk( pos.x, pos.y, 0.2 );
	  }
	  break;
	case b2Shape::e_polygon:
	  {
	    b2PolygonShape* poly = (b2PolygonShape*)f->GetShape();
	    
	    const int count = poly->GetVertexCount();
	    
	    glColor3dv( color );
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
	    //glColor3dv( color );
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


void DrawDisk(double cx, double cy, double r ) 
{ 
  const int num_segments = 32.0 * sqrtf( r );
  
  const double theta = 2 * M_PI / double(num_segments); 
  const double c = cosf(theta);//precalculate the sine and cosine
  const double s = sinf(theta);
  double t;
  
  double x = r; //we start at angle = 0 
  double y = 0; 
  
  glBegin(GL_TRIANGLE_STRIP); 
  for(int ii = 0; ii < num_segments; ii++) 
    { 
      glVertex2f( x + cx, y + cy);
      glVertex2f( cx, cy );
      
      //apply the rotation matrix
      t = x;
      x = c * x - s * y;
      y = s * t + c * y;
    } 

  glVertex2f( r + cx, 0 + cy); // first point again to close disk

  glEnd(); 
}

GuiWorld::GuiWorld( double width, double height ) :
  World( width, height ),
  window(NULL),
  draw_interval( skip ),
  lights_need_redraw( true )  
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

void GuiWorld::Step( double timestep )
{
  if( !paused || step)
    {
      World::Step( timestep );

      step = false;
      // paused = true;
    }

  if( --draw_interval < 1 )
    {
      draw_interval = skip;

      glClearColor( 0.8, 0.8, 0.8, 1.0 ); 
      glClear(GL_COLOR_BUFFER_BIT);	

      // draw the floor
      int count=0;
      glColor3f( 0.7,0.7,0.7 );
      for (double i = 0; i < width ; ++i) {
	for (double j = 0; j < height; ++j) {
	  //if( count++ % 2 == 0) // if i + j is even
	  if( (int(i) + int(j)) % 2 == 0 )
	  glRectf(i, j, i+1, j+1 );    // draw the rectangle
	}
      }

      // draw the walls
      for( int i=0; i<4; i++ )
	DrawBody( boxWall[i], c_gray );

      for( int i=0; i<4; i++ )
	DrawBody( robotWall[i], c_gray );
      
      for( auto& b : boxes )
	DrawBody( b->body, c_gray );
      
      for( auto& r : robots )
	{
	  double col[3];
	  col[0] = (r->charge_max - r->charge) / r->charge_max;
	  col[1] = r->charge / r->charge_max;
	  col[2] = 0;

	  DrawBody( r->body, col );
	  DrawBody( r->bumper, c_darkred );

#if 0
	  double pixels[8];
	  r->GetNeighbors( pixels );

	  //for( int i=0; i<8; i++ )
	  // printf( "%.3f ", pixels[i] );
	  //puts("");
	  
	  // render the sensors
	  const double fov = 2.0 * M_PI;
	  const double pixel_count = 8;
	  double rad_per_pixel = fov / pixel_count;
	  glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );	  
	  glPushMatrix();
			
	  const b2Vec2 pose = r->body->GetPosition();
	  const double angle = r->body->GetAngle();

	  glTranslatef( pose.x, pose.y, 0 );
	  glRotatef( RTOD(angle), 0,0,1 );

	  //printf( "rad per pixel %.4f\n", rad_per_pixel );

	  for( unsigned int p=0; p<pixel_count; p++ )
	    {
	      double angle = -fov/2.0 + (p+0.5) * rad_per_pixel;
	      double dx1 = pixels[p] * cos(angle+rad_per_pixel/2.0);
	      double dy1 = pixels[p] * sin(angle+rad_per_pixel/2.0);
	      double dx2 = pixels[p] * cos(angle-rad_per_pixel/2.0);
	      double dy2 = pixels[p] * sin(angle-rad_per_pixel/2.0);
	      
	      glColor4f( 1,0,0, pixels[p<3.0] ? 0.2 : 0.05 );
	      
	      glBegin( GL_POLYGON );
	      glVertex2f( 0,0 );
	      glVertex2f( dx1, dy1 );
	      glVertex2f( dx2, dy2 );
	      glEnd();                  
	    }	  
#endif
	  
	  glPopMatrix();

	  double* targets = r->GetTargets();

	  //for( int i=0; i<7; i++ )
	  // printf( "%.3f ", targets[i] );
	  //puts("");
	  
	  // render the sensors
	  const double fov = 2.0 * M_PI;
	  const double pixel_count = 7;
	  double rad_per_pixel = fov / pixel_count;
	  glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );	  
	  glPushMatrix();
	  
	  const b2Vec2 pose = r->body->GetPosition();
	  const double angle = r->body->GetAngle();
	  
	  glTranslatef( pose.x, pose.y, 0 );
	  glRotatef( RTOD(angle), 0,0,1 );
	  
	  //printf( "rad per pixel %.4f\n", rad_per_pixel );
	  
	  for( unsigned int p=0; p<pixel_count; p++ )
	    {
	      const double angle = -fov/2.0 + (p+0.5) * rad_per_pixel;
	      const double dx1 = targets[p] * cos(angle+rad_per_pixel/2.0);
	      const double dy1 = targets[p] * sin(angle+rad_per_pixel/2.0);
	      const double dx2 = targets[p] * cos(angle-rad_per_pixel/2.0);
	      const double dy2 = targets[p] * sin(angle-rad_per_pixel/2.0);
	      
	      //if( r->escape )
		glColor4f( 0,1,0, 0.8 );
		//else
		//glColor4f( 1,0,0, 0.6 );

	      glBegin( GL_POLYGON );
	      glVertex2f( 0,0 );
	      glVertex2f( dx1, dy1 );
	      glVertex2f( dx2, dy2 );
	      glEnd();                  
	    }	  
	  
	  glPopMatrix();
	  
	}
      
      // draw a nose on the robot
      glColor3f( 1,1,1 );
      glPointSize( 12 );
      glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
      glBegin( GL_TRIANGLES );

      for( auto& r : robots )
	{      
	  const b2Transform& t = r->body->GetTransform();
	  const double a = t.q.GetAngle();
	  
	  glVertex2f( t.p.x + r->size/2.0 * cos(a),
		      t.p.y + r->size/2.0 * sin(a) );		  
	  glVertex2f( t.p.x + r->size/3.0 * cos(a+0.5),
		      t.p.y + r->size/3.0 * sin(a+0.5) );		  
	  glVertex2f( t.p.x + r->size/3.0 * cos(a-0.5),
		      t.p.y + r->size/3.0 * sin(a-0.5) );		  
	}
      glEnd();
      
      glBegin( GL_LINES );
      glVertex2f( 0,0 );
      glVertex2f( width,0 );
      glVertex2f( 0,0 );
      glVertex2f( 0,height );
      glEnd();
      
      const size_t side = 64;      
      const double dx = width/(double)side;
      const double dy = height/(double)side;      
      
#if 0
      glLineWidth(5);
      glBegin( GL_LINES );
      // draw a sample of the light intensity vector field
      for( double y=0; y<height; y+=height/32.0 )
	for( double x=0; x<width; x+=width/32.0 )
	  {
	    double l = GetLightIntensityAt( x, y );
	    
	    double eps = 0.5;
	    
	    double maxl = l;
	    double maxdx, maxdy;

	    double r = 0.2;
	    // explore a circle around the point
	    for( double a=0; a<2.0*M_PI; a+=M_PI/64.0 )
	      {		
		double dx = r * cos(a);
		double dy = r * sin(a);	     	  
		double brightness = GetLightIntensityAt( x+dx, y+dy );
		
		if( maxl < brightness )
		  {
		    maxl = brightness;
		    maxdx = dx;
		    maxdy = dy;
		  }
	      }
	    
	    glColor3f(0,0,1);
	    glVertex2f( x, y );
	    glColor3f(1,0,0);
	    glVertex2f( x+maxdx, y+maxdy );
	    
	  }
      glEnd();
#endif

      if( lights_need_redraw )
	{
	  lights_need_redraw = false;

	  // draw grid of light intensity
	  //double bright[side][side];
	  
	  bright.resize( side*side );

	  double max = 0;
	  for( int y=0; y<side; y++ )
	    {
	      for( int x=0; x<side; x++ )
		{
		  // find the world position at this grid location  
		  double wx = x * dx + dx/2.0; 
		  double wy = y * dy + dy/2.0; 
		  
		  bright[y*side+x] = GetLightIntensityAt( wx, wy );
		  
		  // keep track of the max for normalizatioon
		  if( bright[y*side+x] > max )
		    max = bright[y*side+x];
		}
	    }
	  
	  // scale to normalize brightness
	  for( int y=0; y<side; y++ )
	    for( int x=0; x<side; x++ )
	      bright[y*side+x] /= (1.5*max); // actually a little less than full alpha
	}
	  
      // draw the light sources  
      for( const auto& l : lights )
	{	
	  glColor4f( 1,1,0,l->intensity  );
	  DrawDisk( l->x, l->y, 0.05 );
	}
      
      for( int y=0; y<side; y++ )
	for( int x=0; x<side; x++ )
	  {
	    // find the world position at this grid location  
	    double wx = x * dx + dx/2.0; 
	    double wy = y * dy + dy/2.0; 
	    
	    glColor4f( 1,1,0, bright[y*side+x] );
	    
	    glRectf( wx-dx/2.0, wy-dy/2.0,
		     wx+dx/2.0, wy+dy/2.0 );
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

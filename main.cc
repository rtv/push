
#include <stdio.h>
#include <getopt.h>
#include <unistd.h> // for usleep(3)

#include <vector>
#include <iostream>

#include "push.hh"

template<class T>
int sign(T x) {
  return (x > 0) - (x < 0);
}

double letterL[] = { 10,6, 10, 7, 10, 8, 10, 9, 10, 10, 11, 10, 12, 10, 13, 10, 14, 10 }; 
class Pusher : public Robot
{
private:
  typedef enum 
    {
      S_PUSH = 0,
      S_BACKUP,
      S_TURN,
      S_COUNT,
      S_ESCAPE
    } control_state_t;
  
  static const double PUSH, BACKUP, TURNMAX;
  static const double SPEEDX, SPEEDA;
  static const double maxspeedx, maxspeeda;
  
  double timeleft;
  control_state_t state;
  double speedx, speeda;

  double lastintensity;
  double latch;

  int count;
  bool escape;

  int phase;

public:
  
  // constructor
  Pusher( World& world, double size, double x, double y, double a  ) : 
    Robot( world, 
	   x,y,a,
	   size,
	   0, // charge start
	   20, // charge max
	   0.4 ), // input efficiency
    //0.1,
    //	   0 ), // stay charged forever 
    state( S_PUSH ),
    timeleft( drand48() * TURNMAX ),
    speedx( 0 ),
    speeda( 0 ),
    lastintensity(0),
    latch(0),
    count( drand48() * 1000.0 ),
    escape(false),
    phase( random() % 50 )
  {
  }
  
  virtual void Update( double timestep )
  {
    if( world.steps % 50 == phase )
      {	
	const double fleft =  GetLightIntensityAt( +0.1, -0.1 );
	const double fright = GetLightIntensityAt( +0.1, +0.1 );
	
	const double bleft =  GetLightIntensityAt( -0.1, -0.1 );
	const double bright = GetLightIntensityAt( -0.1, +0.1 );
	
	speedx = 10.0 * ((fright + fleft) - (bright+bleft));
	speeda = 10.0 * (fright - fleft);
	
	SetSpeed( speedx, 0, speeda );
      }	

    Robot::Update( timestep ); // inherit underlying behaviour to handle charge/discharge    
  }
}; // class Pusher

// static members
const double Pusher::PUSH = 15.0; // seconds
const double Pusher::BACKUP = 0.5;
const double Pusher::TURNMAX = 2;
const double Pusher::SPEEDX = 0.5;
const double Pusher::SPEEDA = M_PI/2.0;
const double Pusher::maxspeedx = 0.5;
const double Pusher::maxspeeda = M_PI/2.0;

int main( int argc, char* argv[] )
{
  double WIDTH = 32;
  double HEIGHT = 32;
  size_t ROBOTS = 128;
  size_t BOXES = 512;
  size_t LIGHTS = 32*32;
  double timeStep = 1.0 / 30.0;
  double robot_size = 0.35;
  double box_size = 0.25;

  /* options descriptor */
  static struct option longopts[] = {
    { "robots",  required_argument,   NULL,  'r' },
    { "boxes",  required_argument,   NULL,  'b' },
    { "robotsize",  required_argument,   NULL,  'z' },
    { "boxsize",  required_argument,   NULL,  's' },
    //	{ "help",  optional_argument,   NULL,  'h' },
    { NULL, 0, NULL, 0 }
  };
  
  int ch=0, optindex=0;  
  while ((ch = getopt_long(argc, argv, "w:h:r:b:s:z:", longopts, &optindex)) != -1)
    {
      switch( ch )
	{
	case 0: // long option given
	  printf( "option %s given", longopts[optindex].name );
          if (optarg)
            printf (" with arg %s", optarg);
          printf ("\n");
	  break;
	case 'w':
	  WIDTH = atof( optarg );
	  break;
	case 'h':
	  HEIGHT = atof( optarg );
	  break;
	case 'r':
	  ROBOTS = atoi( optarg );
	  break;
	case 'b':
	  BOXES = atoi( optarg );
	  break;
	case 'z':
	  robot_size = atof( optarg );
	  break;
	case 's':
	  box_size = atof( optarg );
	  break;
	// case 'h':  
	// case '?':  
	//   puts( USAGE );
	//   exit(0);
	//   break;
	 default:
	   printf("unhandled option %c\n", ch );
	   //puts( USAGE );
	   exit(0);
	}
    }
  
  GuiWorld world( WIDTH, HEIGHT );
  
  for( int i=0; i<BOXES; i++ )
    world.AddBox( new Box( world, Box::SHAPE_RECT, box_size,
  			   WIDTH/4.0 + drand48()*WIDTH*0.5,
  			   HEIGHT/4.0 + drand48()*HEIGHT*0.5,
			   drand48() * M_PI ) );
  
  for( int i=0; i<ROBOTS; i++ )
    {
      double x = WIDTH/2.0;
      double y = HEIGHT/2.0;
      
      while( x > WIDTH*0.2 && x < WIDTH*0.8 && y > HEIGHT*0.2 && y < HEIGHT*0.8 )
	{
	  x = drand48() * WIDTH;	  
	  y = drand48() * HEIGHT;
	}
      
      world.AddRobot( new Pusher( world, robot_size, x,y, drand48() * M_PI ));
    }
  
  // fill the world with a grid of lights, all off
  world.AddLightGrid( sqrt(LIGHTS), sqrt(LIGHTS), 2.0, 0.0 );
  
  const double RADMAX = WIDTH/2.0;
  double RADMIN = 5;//RADMAX-1;

  double radius = RADMAX;
  double delta = 0.3;
  double xdelta = 0;

  double lside = sqrt(LIGHTS);
  double lx = WIDTH / lside; // distance between lights
  double ly = HEIGHT / lside;

  uint64_t maxsteps = 100000L;

  double goalx = WIDTH/2.0;
  double goaly = HEIGHT/2.0;

  /* Loop until the user closes the window */
  while( !world.RequestShutdown() && world.steps < maxsteps )
    {
      if( world.steps % 100 == 1 ) // every now and again
	{ 
	  if( radius < RADMIN ) 
	    {
	      delta = -delta;// * 2.0;
	      //xdelta = 0.1;
	    }

	  else if( radius > RADMAX ) 
	    delta = -delta;//downdelta;

	  const double r2 = radius * radius;
	  
	  for( int x=0; x<lside; x++ )
	    for( int y=0; y<lside; y++ )
	      {
 		const double cx = (x - goalx) * lx;
		const double cy = (y - goaly) * ly;
		const double c2 = cx*cx + cy*cy;
		
		world.SetLightIntensity( x+y*lside, 
					 (fabs( c2 - r2 ) < lside) );
	      }	  
#if 0
	  for( int i=0; i<18; i+=2 )
	    {
	      size_t index = letterL[i] + letterL[i+1] * lside;	    
	      world.SetLightIntensity( index, 1 ); 	    
	    }
#endif
	  
    
	  radius +=  delta;	  	  	  
	  goalx += xdelta;
	}
      
      world.Step( timeStep );	  
    }
  
  printf( "Completed %lu steps.", world.steps );

  return 0;
}

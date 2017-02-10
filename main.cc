
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
    escape(false)
  {
  }
  
  virtual void Update( double timestep )
  {
    // IMPLEMENT ROBOT ROBOT BEHAVIOUR WITH A LITTLE STATE MACHINE
    
    double lleft = GetLightIntensityAt( 0, -0.1 );
    double lright = GetLightIntensityAt( 0, +0.1 );

    // count down to changing control state
    //timeleft -= timestep;

    //count++;
    

    //bool escape = false;

    // if we are pushing and the bump switch goes off or the light is
    // too bright, force a change of control state
    //if( state == S_PUSH && ( GetLightIntensity() > 1.0 || GetBumperPressed() ) )
    //if( state == S_PUSH && GetBumperPressed() )
    
    if( GetBumperPressed() )
      {
	//puts( "BUMPER" );
	// 	//timeleft = 0.0; // end pushing right now
	
       	latch = 75;              
 	//escape = true;
      }
    
    if( --latch > 0 )
      {
	if( latch > 50 )
	  {
	    //puts( "LATCH BACKUP" );
	    speedx = -0.4; // backup
	    speeda = 0.0;  
	  }
	else if( latch > 25 )
	  {
	    //puts( "LATCH TURN" );
	    speedx = 0; 
	    speeda = 0.8;  // turn
	  }
	else
	  {
	    //puts( "LATCH BAIL" );
	    speedx = 0.4; 
	    speeda = 0.0;  // turn
	  }
      }
    else
      {
	//puts( "taxis" );

	const double l = GetLightIntensity();
	const double diff = lright - lleft;
	//printf( "l %f r %f diff %f\n", lleft, lright, diff );
	
	// 32x32 l > 1.5
	// 16x6 l > 0.4
	if( l > 0.4 || fabs(diff) > 0.01 ) // in the light or large diff
	  speedx = 0;
	else
	  speedx = 0.4;//0.5 * l + 0.1;
	
	speeda = diff > 0 ? 0.5 : -0.5;
      }
        
  //printf( "%.2f : %.2f\n", speedx, speeda );
  SetSpeed( speedx, 0, speeda );
    
    // if low on power but charging, stop moving
    //if( charge < charge_max/5.0 && charge_delta > 0 )
    //  SetSpeed( 0,0,0 );

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
  //size_t LIGHTS = 16*16;
  double timeStep = 1.0 / 30.0;
  double robot_size = 0.3;
  double box_size = 0.3;

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
    world.AddBox( new Box( world, Box::SHAPE_HEX, box_size,
			   WIDTH/4.0 + drand48()*WIDTH*0.5,
			   HEIGHT/4.0 + drand48()*HEIGHT*0.5,
			   drand48() * M_PI ) );
  
  //world.AddRobot( new Pusher( world, robot_size, WIDTH/2.0-3, HEIGHT/2.0-4,0 ));

  //if( 0 )
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
  
  //world.AddLight( new Light( WIDTH/2, HEIGHT/2, 2.0, 1.0) );

  const double RADMAX = WIDTH/2.0 * 0.9;
  double RADMIN = 2.5;//RADMAX-1;

  double radius = RADMAX;

  double updelta = 2.0;
  double downdelta = -0.5;
  double delta =  downdelta;

  double lside = sqrt(LIGHTS);
  double lx = WIDTH / lside; // distance between lights
  double ly = HEIGHT / lside;

  uint64_t maxsteps = 10000L;

  /* Loop until the user closes the window */
  while( !world.RequestShutdown() && world.steps < maxsteps )
    {
      if( world.steps % 300 == 1 ) // every now and again
	{ 
	  if( radius < RADMIN ) 
	    {
	      //delta = updelta;
	      //RADMIN--;
	      radius = RADMAX/2.0;
	      delta = 0.5;
	    }

	  else if( radius > RADMAX ) 
	    delta = downdelta;
	  
	  // turn on a random fraction of the lights
	  //for( int i=0; i<LIGHTS; i++ )
	  //world.SetLightIntensity( i, drand48()>0.95 ? 1.0 : 0 );

	  // 	  world.SetLightIntensity( LIGHTS/2+(sqrt(LIGHTS)/2), 1 );

#if 1
	  for( int x=0; x<lside; x++ )
	    for( int y=0; y<lside; y++ )
	      {
		double cx = (x - lside/2.0) * lx;
		double cy = (y - lside/2.0) * ly;
		
		if( fabs(hypot( cx, cy ) - radius) < WIDTH*0.05 )		    
		   world.SetLightIntensity( x+y*sqrt(LIGHTS), 1 );	  
		else
		  world.SetLightIntensity( x+y*sqrt(LIGHTS), 0 );	  
	      }	  
	  
#endif

	  //for( int i=0; i<sizeof(letterL); i+=2 )
	  // world.SetLightIntensity( letterL[i] + letterL[i+1] *sqrt(LIGHTS), 1 ); 	    

	  //world.SetLightIntensity( x, 1 );	  

#if 1
	  if( radius >= RADMAX )
	    {
	      // turn on the lights around the edge
	      for( int x=0; x<sqrt(LIGHTS); x++ )
		{
		  world.SetLightIntensity( x, 1 );	  
		  world.SetLightIntensity( LIGHTS-x-1, 1 );	  
		  world.SetLightIntensity( x, 2 );	  
		  world.SetLightIntensity( LIGHTS-x-1, 2 );	  
		}
	      for( int y=0; y<sqrt(LIGHTS); y++ )
		{
		  world.SetLightIntensity( y*sqrt(LIGHTS), 1 );	  
		  world.SetLightIntensity( LIGHTS-y*sqrt(LIGHTS)-1, 1 );	  
		  world.SetLightIntensity( y*sqrt(LIGHTS), 2 );	  
		  world.SetLightIntensity( LIGHTS-y*sqrt(LIGHTS)-1, 2 );	  
		}
	    }
#endif
    
	  radius +=  delta;	  	  	  
	}
      
      world.Step( timeStep );	  
    }
  
  printf( "Completed %lu steps.", world.steps );

  return 0;
}

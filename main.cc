
#include <getopt.h>
#include <unistd.h> // for usleep(3)

#include <vector>
#include <iostream>

#include "push.hh"

class Pusher : public Robot
{
private:
  typedef enum 
    {
      S_PUSH = 0,
      S_BACKUP,
      S_TURN,
      S_COUNT
    } control_state_t;
  
  static const float PUSH, BACKUP, TURNMAX;
  static const float SPEEDX, SPEEDA;
  static const float maxspeedx, maxspeeda;
  
  float timeleft;
  control_state_t state;
  float speedx, speeda;

public:
  
  // constructor
  Pusher( World& world, float size ) : 
    Robot( world, 
	   drand48() * world.width, // random location
	   drand48() * world.height, 
	   -M_PI + drand48() * 2.0*M_PI,
	   size,
	   100,
	   100 ), 
    state( S_TURN ),
    timeleft( drand48() * TURNMAX ),
    speedx( 0 ),
    speeda( 0 )
  {
  }
  
  virtual void Update( float timestep )
  {
    // IMPLEMENT ROBOT ROBOT BEHAVIOUR WITH A LITTLE STATE MACHINE
    
    // count down to changing control state
    timeleft -= timestep;
    
    // if we are pushing and the bump switch goes off or the light is
    // too bright, force a change of control state
    if( state == S_PUSH && ( GetLightIntensity() > 1.0 || GetBumperPressed() ) )
      {
	timeleft = 0.0; // end pushing right now
      }
    
    if( timeleft <= 0 ) // time to change to another behaviour
      switch( state )
	{
	case S_PUSH:
	  state = S_BACKUP;
	  timeleft = BACKUP;
	  speedx = -SPEEDX;
	  speeda = 0;	 
	  break;
	  
	case S_BACKUP: 
	  state = S_TURN;
	  timeleft = drand48() * TURNMAX;
	  speedx = 0;
	  speeda = SPEEDA;	    
	  break;
	  
	case S_TURN: 
	  state = S_PUSH;
	  timeleft = PUSH;
	  speedx = SPEEDX;
	  speeda = 0;	 
	  break;
	  
	default:
	  std::cout << "invalid control state: " << state << std::endl;
	exit(1);
	}
    
    SetSpeed( speedx, 0, speeda );
    
    // if low on power but charging, stop moving
    //if( charge < charge_max/5.0 && charge_delta > 0 )
    //  SetSpeed( 0,0,0 );

    Robot::Update( timestep ); // inherit underlying behaviour to handle charge/discharge


  }
}; // class Pusher

// static members
const float Pusher::PUSH = 15.0; // seconds
const float Pusher::BACKUP = 0.5;
const float Pusher::TURNMAX = 2.0;
const float Pusher::SPEEDX = 0.5;
const float Pusher::SPEEDA = M_PI/2.0;
const float Pusher::maxspeedx = 0.5;
const float Pusher::maxspeeda = M_PI/2.0;


int main( int argc, char* argv[] )
{
  float WIDTH = 16;
  float HEIGHT = 16;
  size_t ROBOTS = 64;
  size_t BOXES = 512;
  size_t LIGHTS = 256;
  float timeStep = 1.0 / 30.0;
  float robot_size = 0.4;
  float box_size = 0.3;

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
    world.AddBox( new Box( world, Box::SHAPE_HEX, box_size ) );
  
  for( int i=0; i<ROBOTS; i++ )
    world.AddRobot( new Pusher( world, robot_size ) );

  // fill the world with a grid of lights, all off
  world.AddLightGrid( sqrt(LIGHTS), sqrt(LIGHTS), 2.0, 0.0 );
  
  /* Loop until the user closes the window */
  while( !world.RequestShutdown() )
    {
      if( world.steps % 1000 == 1 ) // every now and again
	{ 
	  // turn on a random fraction of the lights
	  for( int i=0; i<LIGHTS; i++ )
	    world.SetLightIntensity( i, drand48()>0.7 ? 1.0 : 0 );

	  // // turn on the lights around the edge
	  // for( int x=0; x<sqrt(LIGHTS); x++ )
	  //   {
	  //     world.SetLightIntensity( x, 1 );	  
	  //     world.SetLightIntensity( LIGHTS-x-1, 1 );	  
	  //   }
	  // for( int y=0; y<sqrt(LIGHTS); y++ )
	  //   {
	  //     world.SetLightIntensity( y*sqrt(LIGHTS), 1 );	  
	  //     world.SetLightIntensity( LIGHTS-y*sqrt(LIGHTS)-1, 1 );	  
	  //   }
	}

      world.Step( timeStep );	  
    }
  
  return 0;
}

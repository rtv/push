
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
  Pusher( World& world ) : 
    Robot( world, 
	   drand48() * world.width, // random location
	   drand48() * world.height, 
	   -M_PI + drand48() * 2.0*M_PI ), 
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
    if( state == S_PUSH && ( GetLightIntensity() > 0.5 || GetBumperPressed() ) )
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
  }
}; // class Pusher

// static members
const float Pusher::PUSH = 10.0; // seconds
const float Pusher::BACKUP = 0.5;
const float Pusher::TURNMAX = 2.0;
const float Pusher::SPEEDX = 0.5;
const float Pusher::SPEEDA = M_PI/2.0;
const float Pusher::maxspeedx = 0.5;
const float Pusher::maxspeeda = M_PI/2.0;


int main( int argc, char* argv[] )
{
  float WIDTH = 8;
  float HEIGHT = 8;
  size_t ROBOTS = 16;
  size_t BOXES = 128;
  float32 timeStep = 1.0 / 30.0;

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
	  Robot::size = atof( optarg );
	  break;
	case 's':
	  Box::size = atof( optarg );
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
  
  std::vector<Box*> boxes;    
  for( int i=0; i<BOXES; i++ )
    boxes.push_back( new Box( world, Box::SHAPE_HEX ) );
  
  std::vector<Robot*> robots;
  for( int i=0; i<ROBOTS; i++ )
    robots.push_back( new Pusher( world ) );
    
  /* Loop until the user closes the window */
  while( !world.RequestShutdown() )
    {
      // if( ! GuiWorld::paused )
      for( int i=0; i<ROBOTS; i++ )
	robots[i]->Update( timeStep );
      
      world.Step( timeStep, robots, boxes );
    }
  
  return 0;
}

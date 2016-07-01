#include "robot.hh"

#include <iostream>

// static members
const float DemoPusher::PUSH = 10.0; // seconds
const float DemoPusher::BACKUP = 0.5;
const float DemoPusher::TURNMAX = 3.0;
const float DemoPusher::SPEEDX = 0.5;
const float DemoPusher::SPEEDA = M_PI/2.0;

// constructor
DemoPusher::DemoPusher( void ) : 
  Ctrl(), 
  state( S_TURN ),
  timeleft( drand48() * TURNMAX ),
  speedx( 0 ),
  speeda( 0 )
{
}

// called once after the host robot is intialized
void DemoPusher::Init( Robot& bot )  
{
  // just in case we need to inspect the robot to get set up
  // (nothing to do in this case)
}

// called once per simulation step, of duration timestep seconds
void DemoPusher::Update( Robot& bot, float timestep )
{
  // IMPLEMENT ROBOT ROBOT BEHAVIOUR WITH A LITTLE STATE MACHINE
  
  // count down to changing control state
  timeleft -= timestep;
  
  // if we are pushing and the bump switch goes off or the light is
  // too bright, force a change of control state
  if( state == S_PUSH && ( bot.GetLightIntensity() > 0.5 || bot.GetBumperPressed() ) )
    {
      timeleft = 0; // end pushing right now      
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

  bot.SetSpeed( speedx, 0, speeda );
}

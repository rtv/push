#include <iostream>

#include "robot.hh"

// static members
const float Robot::PUSH = 5.0; // seconds
const float Robot::BACKUP = 1.0;
const float Robot::TURNMAX = 2.0;
const float Robot::maxspeedx = 0.5;
const float Robot::maxspeeda = M_PI/2.0;
const float Robot::size = 0.3;

// constructor
Robot::Robot( b2World& world, float x, float y, float a ) : 
  pushTime( PUSH ),
  backupTime( BACKUP ),
  turnTime( drand48() * TURNMAX ),
  state( (control_state_t)(random() % S_COUNT) ), // choose start state at random
  speedx( 0 ),
  speeda( 0 ),
  body( NULL )
{
  b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;
  body = world.CreateBody(&bodyDef);
  
  b2PolygonShape dynamicBox;
  dynamicBox.SetAsBox( size/2.0, size/2.0 );
  
  b2FixtureDef fixtureDef;
  fixtureDef.shape = &dynamicBox;    
  fixtureDef.density = 10;
  fixtureDef.friction = 2.0;
  body->CreateFixture(&fixtureDef);
  
  // place in the world
  body->SetTransform( b2Vec2( x, y ), a );	
  
  // any need for this?
  //bodies[0]->SetLinearDamping( 5.0 );
  //bodies[0]->SetAngularDamping( 10.0 );
}

// called once per simulation step, of duration timestep seconds
void Robot::Update( float timestep )
{
  // implement the robot behaviour with a little state machine
  switch( state )
    {
    case S_PUSH: // push
      speedx = maxspeedx;
	speeda = 0;	 
	pushTime -= timestep;
	//std::cout << "Pushing " << pushTime << std::endl;
	if( pushTime <= 0 )
	  {
	    state = S_BACKUP;
	    pushTime = PUSH;
	  }
	break;
	
    case S_BACKUP: // backup
      speedx = -maxspeedx;
      speeda = 0;	    
      backupTime -= timestep;
      //std::cout << "Backup " << backupTime << std::endl;
      if( backupTime <= 0 )
	{
	  state = S_TURN;
	  backupTime = BACKUP;
	}
      break;
      
    case S_TURN: // turn
      speedx = 0;
      speeda = maxspeeda;	    
      turnTime -= timestep;
      //std::cout << "Turning " << turnTime << std::endl;
      if( turnTime <= 0 )
	{
	  state = S_PUSH;
	  turnTime = drand48() * TURNMAX;
	}
      break;
    default:
      std::cout << "invalid control state: " << state << std::endl;
      exit(1);
    }
  
  // set body speed in body-local coordinate frame
  body->SetLinearVelocity( body->GetWorldVector(b2Vec2( speedx, 0 )));
  body->SetAngularVelocity( speeda );
}


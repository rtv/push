#include <iostream>

#include "robot.hh"

// static members
const float Robot::PUSH = 10.0; // seconds
const float Robot::BACKUP = 0.5;
const float Robot::TURNMAX = 3.0;
const float Robot::SPEEDX = 0.5;
const float Robot::SPEEDA = M_PI/2.0;

std::vector<Light> Robot::lights(2);

float Robot::SIZE = 0.15;

// constructor
Robot::Robot( b2World& world, const float x, const float y, const float a ) : 
  pushTime( PUSH ),
  backupTime( BACKUP ),
  turnTime( drand48() * TURNMAX ),
  state( (control_state_t)(random() % S_COUNT) ), // choose start state at random
  speedx( 0 ),
  speeda( 0 ),
  body( NULL ),
  joint( NULL )
{
  lights[0].x = 2;
  lights[0].y = 2;
  lights[0].intensity = 0.7;

  lights[1].x = 5;
  lights[1].y = 5;
  lights[1].intensity = 0.8;

  
  b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;

  //bodyDef.position.Set( 0,0 );
  body = world.CreateBody(&bodyDef);

  //bodyDef.position.Set( SIZE/2.0, 0 );
  bumper = world.CreateBody(&bodyDef);
  
  b2PolygonShape dynamicBox;
  dynamicBox.SetAsBox( SIZE/2.0, SIZE/2.0 );
  
  b2FixtureDef fixtureDef;
  fixtureDef.shape = &dynamicBox;    
  fixtureDef.density = 10;
  fixtureDef.friction = 1.0;
  body->CreateFixture(&fixtureDef);
  
  // bumper has same settings the body but different size
  dynamicBox.SetAsBox( SIZE/10.0, SIZE/2.0 );
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
  jointDef.localAnchorA.Set( SIZE/2.0, 0); // on the nose
  jointDef.localAnchorB.Set( 0,0 );
  
  jointDef.enableMotor = true;
  //jointDef.collideConnected = true;
  
  joint = (b2PrismaticJoint*)world.CreateJoint( &jointDef );

  // place assembled robot in the world
  body->SetTransform( b2Vec2( x, y ), a );	
  bumper->SetTransform( body->GetWorldPoint( b2Vec2( SIZE/2,0) ), a );	


  // any need for this?
  //bodies[0]->SetLinearDamping( 5.0 );
  //bodies[0]->SetAngularDamping( 10.0 );
}

// called once per simulation step, of duration timestep seconds
void Robot::Update( float timestep )
{
  // light intensity triggers push
  b2Vec2 here = body->GetWorldCenter();


  // integrate brightness over all light sources
  float brightness = 0.0;
  for( std::vector<Light>::iterator it = Robot::lights.begin(); 
       it != Robot::lights.end(); 
       it++ )
    {
      float distanceToLightSqrd = 
	pow( here.x - it->x, 2.0 ) + 
	pow( here.y - it->y, 2.0 );
      
      brightness += it->intensity / distanceToLightSqrd;
    }
      //std::cout << "brightness: " << brightness << std::endl;

  // implement the robot behaviour with a little state machine
  switch( state )
    {
    case S_PUSH: // push
      
      if( brightness > 0.5 || joint->GetJointTranslation() < 0.01 )
	{
	  pushTime = 0; // end pushing right now      
	}
      
      speedx = SPEEDX;
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
      speedx = -SPEEDX;
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
      speeda = SPEEDA;	    
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


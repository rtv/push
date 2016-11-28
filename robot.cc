#include "push.hh"


// constructor
Robot::Robot( World& world, 
	      double x, 
	      double y, 
	      double a, 
	      double size,
	      double charge,
	      double charge_max,
	      double input_efficiency,
	      double output_metabolic, 
	      double output_efficiency ) : 
  world(world),
  size( size ),
  charge(charge),
  charge_max(charge_max),
  charge_delta(0),
  input_efficiency(input_efficiency),
  output_metabolic(output_metabolic),
  output_efficiency(output_efficiency),
  body( NULL ),
  bumper( NULL ),
  joint( NULL )
{
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

  // prevent collision with puck-retaining strings 
  fixtureDef.filter.categoryBits = ROBOT;
  fixtureDef.filter.maskBits = ROBOT | BOX | ROBOTBOUNDARY; // not box boundary

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

double Robot::GetLightIntensity( void )
{
  const b2Vec2 here = body->GetWorldCenter();    
  return world.GetLightIntensityAt( here.x, here.y );
}

bool Robot::GetBumperPressed( void )
{
  return( joint->GetJointTranslation() < 0.01 );
}

// set body speed in body-local coordinate frame
void Robot::SetSpeed( double x, double y, double a )
{  
  body->SetLinearVelocity( body->GetWorldVector(b2Vec2( x, y )));
  body->SetAngularVelocity( a );
}

void Robot::Update( double timestep )
{
  // absorb energy from lights
  charge_delta = input_efficiency * GetLightIntensity(); // gather power from light

  // expend energy just living  
  charge_delta -= output_metabolic; 
  
  // expend energy by moving
  charge_delta -= output_efficiency * body->GetAngularVelocity(); 
  charge_delta -= output_efficiency * body->GetLinearVelocity().Length();

  //  charge += charge_delta;
  
  if( charge <= 0.0 )
    {
      charge = 0.0;
      SetSpeed( 0,0,0 ); // can't move
    }
  else if( charge > charge_max ) 
    charge = charge_max; // full up
}

#include "push.hh"

World::World( float width, float height ) :
  steps(0),
  width(width),
  height(height),
  b2world( new b2World( b2Vec2( 0,0 ))), // gravity 
  lights() //empty vector
{
    // set interior box container
    b2BodyDef boxWallDef;
    b2PolygonShape groundBox;
    groundBox.SetAsBox( width/3.0, 0.01f );    
    
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &groundBox;    
    
    // prevent collision with puck-retaining strings 
    fixtureDef.filter.categoryBits = BOXBOUNDARY;
    fixtureDef.filter.maskBits = BOX; // contain only boxes
  
    for( int i=0; i<4; i++ )
      {
	boxWall[i] = b2world->CreateBody(&boxWallDef);	
	boxWall[i]->CreateFixture(&fixtureDef );
      }
    
    boxWall[0]->SetTransform( b2Vec2( width/2, height/6.0 ), 0 );    
    boxWall[1]->SetTransform( b2Vec2( width/2, height-height/6.0 ), 0 );    
    boxWall[2]->SetTransform( b2Vec2( width/6.0, height/2 ), M_PI/2.0 );    
    boxWall[3]->SetTransform( b2Vec2( width-width/6.0, height/2 ), M_PI/2.0 );


    // set exterior box container
    b2BodyDef boxWallDef1;
    b2PolygonShape groundBox1;
    groundBox1.SetAsBox( width, 0.01f );    
    
    b2FixtureDef fixtureDef1;
    fixtureDef1.shape = &groundBox1;    
    
    // prevent collision with puck-retaining strings 
    fixtureDef1.filter.categoryBits = ROBOTBOUNDARY;
    fixtureDef1.filter.maskBits = ROBOT | BOX; // contain everthing
  
    for( int i=0; i<4; i++ )
      {
	robotWall[i] = b2world->CreateBody(&boxWallDef1);	
	robotWall[i]->CreateFixture(&fixtureDef1 );
      }
    
    robotWall[0]->SetTransform( b2Vec2( width/2,0 ), 0 );    
    robotWall[1]->SetTransform( b2Vec2( width/2,height ), 0 );    
    robotWall[2]->SetTransform( b2Vec2( 0, height/2 ), M_PI/2.0 );    
    robotWall[3]->SetTransform( b2Vec2( width, height/2 ), M_PI/2.0 ); 
}


void World::AddLight( Light* l )
{
  lights.push_back( l );
}

void World::AddLightGrid( size_t xcount, size_t ycount, float z, float intensity )
{
  float xspace = width/(float)xcount;
  float yspace = height/(float)ycount;
  
  for( size_t y=0; y<ycount; y++ )
    for( size_t x=0; x<xcount; x++ )
      AddLight( new Light(  x * xspace + xspace/2.0, 
			    y * yspace + yspace/2.0,
			    z,
			    intensity ) );
}

void World::AddRobot( Robot* r  )
{
  robots.push_back( r );
}

void World::AddBox( Box* b  )
{
  boxes.push_back( b );
}

void World::SetLightIntensity( size_t index, float intensity )
{
  if( index < lights.size() )
    lights[index]->intensity = intensity;
}

float World::GetLightIntensityAt( float x, float y )
{
  // integrate brightness over all light sources
  float total_brightness = 0.0;

  for( auto& l : lights )
    {
      // horizontal and vertical distances
      float dx = x - l->x;
      float dy = y - l->y;
      float dz = l->z;

      float distsquared = dx*dx + dy*dy + dz*dz;
      float dist = sqrt( distsquared );

      // brightness as a function of distance
      float brightness = l->intensity / distsquared;

      // now factor in the angle to the light      
      float theta = atan2( dz, hypot(dx*dx,dy*dy) );

      // and integrate
      total_brightness += brightness * sin(theta);
    }

  return total_brightness;
}


void World::Step( float timestep )
{
  for( auto& r : robots )
    r->Update( timestep );

  const int32 velocityIterations = 6;
  const int32 positionIterations = 2;
    
  // Instruct the world to perform a single step of simulation.
  // It is generally best to keep the time step and iterations fixed.
  b2world->Step( timestep, velocityIterations, positionIterations);	

  steps++;
}

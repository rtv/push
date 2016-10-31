#include <iostream>

#include "push.hh" 

Box::Box( World& world, box_shape_t shape, float size )
  : body(NULL),
    size(size)
{ 
  b2PolygonShape dynamicBox;
  
  switch( shape )
    {	    
    case SHAPE_RECT:
      dynamicBox.SetAsBox( size/2.0, size/2.0 );
      break;
    case SHAPE_HEX:
      {
	b2Vec2 verts[6];
	
	for (int i = 0; i < 6; i++) {
	  verts[i].x = size/2.0 * cos(2.0 * M_PI * i / 6.0);
	  verts[i].y = size/2.0 * sin(2.0 * M_PI * i / 6.0);
	}
	
	dynamicBox.Set(verts,6);
      }
      break;
    default:
      std::cout << "invalid shape number " << shape << std::endl;
      break;
    }
  
  b2FixtureDef fixtureDef;
  fixtureDef.shape = &dynamicBox;
  fixtureDef.density = 2;
  fixtureDef.friction = 1.0;
  fixtureDef.restitution = 0.1;
  
  b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;
  
  body = world.b2world->CreateBody(&bodyDef);    
  body->SetLinearDamping( 10.0 );
  body->SetAngularDamping( 10.0 );
  body->SetTransform( b2Vec2( world.width * drand48(), 
			      world.height * drand48()),
		      0 );	    	    
  
  body->CreateFixture(&fixtureDef);
}

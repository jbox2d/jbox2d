/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

//#ifndef B2_COLLISION_H
//#define B2_COLLISION_H

//#include "../Common/b2Math.h"

//struct b2CircleShape;
//struct b2PolyShape;

//Java note: hmm...figure out what to do here...see b2CollidePoly for example use of this class
// We use contact ids to facilitate warm starting.
class b2ContactID{
	public b2ContactID(){
		
	}
	
	//Needed for copying of contact id
	public b2ContactID(b2ContactID copyMe){
		
	}
	
	struct Features
	{
		uint8 referenceFace;
		uint8 incidentEdge;
		uint8 incidentVertex;
		uint8 flip;
	} features;
	uint32 key;
};

class b2ContactPoint{
	public b2Vec2 position;
	public float32 separation;
	public float32 normalImpulse;
	public float32 tangentImpulse;
	public b2ContactID id;
	
	public b2ContactPoint(){
		position = new b2Vec2(0f,0f);
		id = new b2ContactID();
	}
	
};

// A manifold for two touching convex shapes.
class b2Manifold{
	public b2ContactPoint points[b2_maxManifoldPoints];
	public b2Vec2 normal;
	public int32 pointCount;
	
	public b2Manifold(){
		points = new b2ContactPoint[b2_maxManifoldPoints];
		normal = new b2Vec2(0f,0f);
		pointCount = 0;
		for (int i=0; i<b2_maxManifoldPoints; i++){
			points[i] = new b2ContactPoint();
		}
	}
	
};

class b2AABB{
	public boolean IsValid(){
		b2Vec2 d = b2Math.subtract(maxVertex, minVertex);
		boolean valid = d.x >= 0.0f && d.y >= 0.0f;
		valid = valid && minVertex.IsValid() && maxVertex.IsValid();
		return valid;
	}

	b2Vec2 minVertex, maxVertex;
	
	public b2AABB(){
		minVertex = new b2Vec2(0f,0f);
		maxVertex = new b2Vec2(0f,0f);
	}
	
	public b2AABB(b2Vec2 minV, b2Vec2 maxV){
		minVertex = new b2Vec2(minV);
		maxVertex = new b2Vec2(maxV);
	}
	
};

//void b2CollideCircle(b2Manifold* manifold, b2CircleShape* circle1, b2CircleShape* circle2);
//void b2CollidePolyAndCircle(b2Manifold* manifold, const b2PolyShape* poly, const b2CircleShape* circle);
//void b2CollidePoly(b2Manifold* manifold, const b2PolyShape* poly1, const b2PolyShape* poly2);


//Java note: global function, figure out what to do with it...
public boolean b2TestOverlap(b2AABB a, b2AABB b){
	b2Vec2 d1, d2;
	d1 = b2Math.subtract(b.minVertex, a.maxVertex);
	d2 = b2Math.subtract(a.minVertex, b.maxVertex);

	if (d1.x > 0.0f || d1.y > 0.0f)
		return false;

	if (d2.x > 0.0f || d2.y > 0.0f)
		return false;

	return true;
}

//#endif

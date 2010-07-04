/*
* Copyright (c) 2007 Erin Catto http://www.gphysics.com
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

//#include "b2Collision.h"
//#include "b2Shape.h"

//Java note: both of these are global functions...

void b2CollideCircle(b2Manifold manifold, b2CircleShape circle1, b2CircleShape circle2){
	manifoldpointCount = 0;

	b2Vec2 d = b2Math.subtract(circle2.m_position, circle1.m_position);
	float32 distSqr = b2Math.b2Dot(d, d);
	float32 radiusSum = circle1.m_radius + circle2.m_radius;
	if (distSqr > radiusSum * radiusSum){
		return;
	}

	float32 separation;
	if (distSqr < FLT_EPSILON){
		separation = -radiusSum;
		manifold.normal.Set(0.0f, 1.0f);
	} else{
		float32 dist = sqrtf(distSqr); //Java note: I forget if I decided to map sqrt calls not to ...figure out later
		separation = dist - radiusSum;
		float32 a = 1.0f / dist;
		manifold.normal.x = a * d.x;
		manifold.normal.y = a * d.y;
	}

	manifold.pointCount = 1;
	manifold.points[0].id.key = 0;
	manifold.points[0].separation = separation;
	manifold.points[0].position = b2Math.subtract(circle2.m_position, b2Math.multiply(circle2.m_radius, manifold.normal));
}

void b2CollidePolyAndCircle(b2Manifold manifold, b2PolyShape poly, b2CircleShape circle){
	manifold.pointCount = 0;

	// Compute circle position in the frame of the polygon.
	b2Vec2 xLocal = b2Math.b2MulT(poly.m_R, b2Math.subtract(circle.m_position, poly.m_position));

	// Find edge with maximum separation.
	int32 normalIndex = 0;
	float32 separation = -FLT_MAX;
	for (int32 i = 0; i < poly.m_vertexCount; ++i){
		float32 s = b2Math.b2Dot(poly.m_normals[i], b2Math.subtract(xLocal, poly.m_vertices[i]));
		if (s > circle.m_radius){
			// Early out.
			return;
		}

		if (s > separation){
			normalIndex = i;
			separation = s;
		}
	}

	manifold.pointCount = 1;
	manifold.normal = b2Math.b2Mul(poly.m_R, poly.m_normals[normalIndex]);
	manifold.points[0].id.key = 0;
	manifold.points[0].position = b2Math.subtract(circle.m_position, b2Math.multiply(circle.m_radius, manifold.normal));
	manifold.points[0].separation = separation - circle.m_radius;
}

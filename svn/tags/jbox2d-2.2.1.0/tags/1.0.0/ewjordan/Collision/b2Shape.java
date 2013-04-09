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

//#include "b2Shape.h"
//#include "../Dynamics/b2Body.h"
//#include "../Dynamics/b2World.h"
//#include "../Common/b2BlockAllocator.h"
//#include <new.h>

//#ifndef B2_SHAPE_H
//#define B2_SHAPE_H

//#include "../Common/b2Math.h"

//struct b2Body;

final class b2MassData{
	//Java note: add a no arg constructor, please
	public float32 mass;
	public b2Vec2 center;
	public float32 I;
	
	public b2MassData(){
		mass = 0.0f;
		center = new b2Vec2(0.0f,0.0f);
		I = 0.0f;
	}
	
}

//Java note: should we use Java enum or not?  Maybe just final ints are better...
enum b2ShapeType
{
	e_unknownShape = -1,
	e_circleShape,
	e_boxShape,
	e_polyShape,
	e_meshShape,
	e_shapeTypeCount,
};

class b2ShapeDef{
	public b2ShapeDef(){
		type = e_unknownShape;
		userData = null;
		localPosition = new b2Vec2(0.0f, 0.0f);
		localRotation = 0.0f;
		friction = 0.2f;
		restitution = 0.0f;
		density = 0.0f;
		categoryBits = 0x0001;
		maskBits = 0xFFFF;
		groupIndex = 0;
	}

	public void b2ShapeDefDestructor() {}

	void ComputeMass(b2MassData* massData) const;
	public void ComputeMass(b2MassData massData){
		if (density == 0.0f){
			massData.mass = 0.0f;
			massData.center.Set(0.0f, 0.0f);
			massData.I = 0.0f;
		}
		
		switch (type){
			case e_circleShape:
				{
					b2CircleDef circle = (b2CircleDef)this;
					massData.mass = density * b2_pi * circle.radius * circle.radius;
					massData.center.Set(0.0f, 0.0f);
					massData.I = 0.5f * (massData.mass) * circle.radius * circle.radius;
				}
				break;
				
			case e_boxShape:
				{
					b2BoxDef box = (b2BoxDef)this;
					massData.mass = 4.0f * density * box.extents.x * box.extents.y;
					massData.center.Set(0.0f, 0.0f);
					massData.I = massData->mass / 3.0f * b2Dot(box.extents, box.extents);
				}
				break;
				
			case e_polyShape:
				{
					b2PolyDef poly = (b2PolyDef)this;
					PolyMass(massData, poly.vertices, poly.vertexCount, density);
				}
				break;
				
			default:
				massData.mass = 0.0f;
				massData.center.Set(0.0f, 0.0f);
				massData.I = 0.0f;
				break;
		}
	}
	

	b2ShapeType type;
	Object userData;
	b2Vec2 localPosition;
	float32 localRotation;
	float32 friction;
	float32 restitution;
	float32 density;

	// The collision category bits. Normally you would just set one bit.
	uint16 categoryBits;

	// The collision mask bits. This states the categories that this
	// shape would accept for collision.
	uint16 maskBits;

	// Collision groups allow a certain group of objects to never collide (negative)
	// or always collide (positive). Zero means no collision group. Non-zero group
	// filtering always wins against the mask bits.
	int16 groupIndex;
};

class b2CircleDef extends b2ShapeDef{
	public b2CircleDef(){
		super();
		type = e_circleShape;
		radius = 1.0f;
	}

	float32 radius;
};

class b2BoxDef extends b2ShapeDef{
	public b2BoxDef(){
		super();
		type = e_boxShape;
		extents = new b2Vec2(1.0f, 1.0f);
	}

	b2Vec2 extents;
};

// Convex polygon, vertices must be in CCW order.
class b2PolyDef extends b2ShapeDef{
	public b2PolyDef(){
		super();
		type = e_polyShape;
		vertexCount = 0;
		vertices = new b2Vec2[b2_maxPolyVertices];
	}

	b2Vec2 vertices[b2_maxPolyVertices];
	int32 vertexCount;
};

// Shapes are created automatically when a body is created.
// Client code does not normally interact with shapes.
abstract class b2Shape{
	public boolean TestPoint(b2Vec2 p);
	
	public Object GetUserData(){
		return m_userData;
	}

	public b2ShapeType GetType(){
		return m_type;
	}

	// Get the parent body of this shape.
	public b2Body GetBody(){
		return m_body;
	}

	public b2Vec2 GetPosition(){
		return m_position;
	}
	
	
	public float32 GetRotation(){
		return m_rotation;
	}
	
	public b2Mat22 GetRotationMatrix();{
		return m_R;
	}

	// Get the next shape in the parent body's shape list.
	public b2Shape GetNext(){
		return m_next;
	}

	//--------------- Internals Below -------------------

	// Internal use only. Do not call.
	public static b2Shape Create( b2ShapeDef def,
							 b2Body body, b2Vec2 center){
		switch (def.type){
			case e_circleShape:
				{
					return new b2CircleShape(def, body, center);
				}
				
			case e_boxShape:
			case e_polyShape:
				{
					return new b2PolyShape(def, body, center);
				}
		}
		
		b2Assert(false);
		return NULL;
	}

	// Internal use only. Do not call.
	public static void Destroy(b2Shape shape){
		//b2BlockAllocator& allocator = shape->m_body->m_world->m_blockAllocator;
		
		switch (shape.m_type)
		{
			case e_circleShape:
				shape.b2ShapeDestructor();
				//allocator.Free(shape, sizeof(b2CircleShape));
				break;
				
			case e_polyShape:
				shape.b2ShapeDestructor();
				//allocator.Free(shape, sizeof(b2PolyShape));
				break;
				
			default:
				b2Assert(false);
		}
		
		shape = null;
	}

	// Internal use only. Do not call.
	public b2Shape(b2ShapeDef def, b2Body body, b2Vec2 center){
		m_userData = def.userData;
		m_localPosition = b2Math.subtract(def.localPosition,center);
		m_localRotation = def.localRotation;
		m_friction = def.friction;
		m_restitution = def.restitution;
		m_body = body;
		
		m_position = b2Math.add(m_body.m_position, b2Math.b2Mul(m_body.m_R, m_localPosition));
		m_rotation = m_body.m_rotation + m_localRotation;
		m_R.Set(m_rotation);
		
		m_proxyId = b2_nullProxy;
	}

	// Internal use only. Do not call.
	public abstract void b2ShapeDestructor(){
		m_body.m_world.m_broadPhase.DestroyProxy(m_proxyId);
	}

	// Internal use only. Do not call.
	public abstract void UpdateProxy();

	public b2ShapeType m_type;

	public Object m_userData;

	public b2Body m_body;
	public uint16 m_proxyId;

	// Position in world
	public b2Vec2 m_position;
	public float32 m_rotation;
	public b2Mat22 m_R;

	// Local position in parent body
	public b2Vec2 m_localPosition;
	public float32 m_localRotation;

	public float32 m_friction;
	public float32 m_restitution;

	public b2Shape m_next;
};

class b2CircleShape extends b2Shape{
	public boolean TestPoint(b2Vec2 p){
		b2Vec2 d = b2Math.subtract(p, m_position);
		return (b2Math.b2Dot(d, d) <= m_radius * m_radius);
	}

	//--------------- Internals Below -------------------

	public b2CircleShape(b2ShapeDef def, b2Body body, b2Vec2 center){
			super(def, body, center);
			b2Assert(def.type == e_circleShape);
			b2CircleDef circle = (b2CircleDef)def;
			
			m_type = e_circleShape;
			m_radius = circle.radius;
			
			b2AABB aabb = new b2AABB();
			aabb.minVertex.Set( b2Math.subtract(m_position.x, m_radius), b2Math.subtract(m_position.y, m_radius));
			aabb.maxVertex.Set( b2Math.add(m_position.x, m_radius), b2Math.add(m_position.y, m_radius));
			
			m_proxyId = m_body.m_world.m_broadPhase.CreateProxy(aabb, def.groupIndex, def.categoryBits, def.maskBits, this);
	}
	

	public void b2CircleShape::UpdateProxy(){
		b2AABB aabb = new b2AABB;
		aabb.minVertex.Set( b2Math.subtract(m_position.x, m_radius), b2Math.subtract(m_position.y, m_radius));
		aabb.maxVertex.Set( b2Math.add(m_position.x, m_radius), b2Math.add(m_position.y, m_radius));
		
		m_body.m_world.m_broadPhase.MoveProxy(m_proxyId, aabb);
	}
	

	public float32 m_radius;
};

class b2PolyShape extends b2Shape{
	bool TestPoint(const b2Vec2& p);
	bool b2PolyShape::TestPoint(const b2Vec2& p)
	{
		b2Vec2 pLocal = b2MulT(m_R, p - m_position);
		
		for (int32 i = 0; i < m_vertexCount; ++i)
		{
			float32 dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
			if (dot > 0.0f)
			{
				return false;
			}
		}
		
		return true;
	}
	
	//--------------- Internals Below -------------------
	
	//Java note: TONS of inlining is possible here, basically to get rid of all
	//function call overhead.  But this is just a constructor, so it's not really
	//part of the overall sim. loop, and efforts would be better spent elsewhere.
	public b2PolyShape(b2ShapeDef def, b2Body body, b2Vec2 center){
			super(def, body, center);
			b2Assert(def.type == e_boxShape || def.type == e_polyShape);
			m_type = e_polyShape;
			m_vertices = new b2Vec2[b2_maxPolyVertices];
			m_normals = new b2Vec2[b2_maxPolyVertices];
			m_next = new int32[b2_maxPolyVertices];
			m_extents = new b2Vec2(0f,0f);
			
			if (def.type == e_boxShape){
				b2BoxDef box = (b2BoxDef)def;
				m_vertexCount = 4;
				b2Vec2 h = box.extents;
				m_vertices[0] = new b2Vec2(h.x, h.y);
				m_vertices[1] = new b2Vec2(-h.x, h.y);
				m_vertices[2] = new b2Vec2(-h.x, -h.y);
				m_vertices[3] = new b2Vec2(h.x, -h.y);
				m_normals[0] = new b2Vec2(0.0f, 1.0f);
				m_normals[1] = new b2Vec2(-1.0f, 0.0f);
				m_normals[2] = new b2Vec2(0.0f, -1.0f);
				m_normals[3] = new b2Vec2(1.0f, 0.0f);
				m_next[0] = 1;
				m_next[1] = 2;
				m_next[2] = 3;
				m_next[3] = 0;
				
				m_extents = h;
			}
			else{
				b2PolyDef poly = (b2PolyDef)def;
				b2AABB aabb = new b2AABB();
				aabb.minVertex.Set(FLT_MAX, FLT_MAX);
				aabb.maxVertex.Set(-FLT_MAX, -FLT_MAX);
				m_vertexCount = poly.vertexCount;
				b2Assert(3 <= m_vertexCount && m_vertexCount <= b2_maxPolyVertices);
				for (int32 i = 0; i < m_vertexCount; ++i){
					m_vertices[i] = new b2Vec2(poly.vertices[i]);
					
					aabb.minVertex = b2Min(aabb.minVertex, m_vertices[i]);
					aabb.maxVertex = b2Max(aabb.maxVertex, m_vertices[i]);
				}
				b2Vec2 offset = b2Math.multiply(0.5f, b2Math.add(aabb.minVertex, aabb.maxVertex));
				
				b2Assert(m_localRotation == 0.0f); // TODO_ERIN handle local rotation
				
				m_localPosition.add(offset);
				for (int32 i = 0; i < m_vertexCount; ++i){
					// Shift the vertices so the shape position is the centroid.
					m_vertices[i] = b2Math.subtract(poly.vertices[i], offset);
					m_next[i] = i + 1 < m_vertexCount ? i + 1 : 0;
					b2Vec2 vNext = b2Math.subtract(poly.vertices[m_next[i]], offset);
					b2Vec2 edge = b2Math.subtract(vNext, m_vertices[i]);
					m_normals[i] = b2Cross(edge, 1.0f);
					m_normals[i].Normalize();
				}
				
				for (int32 i = 0; i < m_vertexCount; ++i){
					// Ensure the polygon is convex.
					b2Assert(b2Cross(m_normals[i], m_normals[m_next[i]]) > 0.0f);
				}
				
				m_extents = b2Math.multiply(0.5f, b2Math.subtract(aabb.maxVertex, aabb.minVertex));
			}
			
			b2Mat22 absR = b2Math.b2Abs(m_R);
			b2Vec2 h = b2Math.b2Mul(absR, m_extents);
			b2AABB aabb = new b2AABB();
			aabb.minVertex = b2Math.subtract(m_position, h);
			aabb.maxVertex = b2Math.add(m_position, h);
			m_proxyId = m_body.m_world.m_broadPhase.CreateProxy(aabb, def.groupIndex, def.categoryBits, def.maskBits, this);
	}
	

	public void UpdateProxy(){
		b2Mat22 absR = b2Math.b2Abs(m_R);
		b2Vec2 h = b2Math.b2Mul(absR, m_extents);
		b2AABB aabb = new b2AABB();
		aabb.minVertex = b2Math.subtract(m_position, h);
		aabb.maxVertex = b2Math.add(m_position, h);
		m_body.m_world.m_broadPhase.MoveProxy(m_proxyId, aabb);
	}
	

	b2Vec2 m_extents;
	b2Vec2 m_vertices[];
	int32 m_vertexCount;
	b2Vec2 m_normals[];
	int32 m_next[];
};

// Polygon mass, centroid, and inertia.
// Let rho be the polygon density in mass per unit area.
// Then:
// mass = rho * int(dA)
// centroid.x = (1/mass) * rho * int(x * dA)
// centroid.y = (1/mass) * rho * int(y * dA)
// I = rho * int((x*x + y*y) * dA)
//
// We can compute these integrals by summing all the integrals
// for each triangle of the polygon. To evaluate the integral
// for a single triangle, we make a change of variables to
// the (u,v) coordinates of the triangle:
// x = x0 + e1x * u + e2x * v
// y = y0 + e1y * u + e2y * v
// where 0 <= u && 0 <= v && u + v <= 1.
//
// We integrate u from [0,1-v] and then v from [0,1].
// We also need to use the Jacobian of the transformation:
// D = cross(e1, e2)
//
// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
//
// The rest of the derivation is handled by computer algebra.

//Java note: figure out where to put this...
static void PolyMass(b2MassData massData, b2Vec2 vs, int32 count, float32 rho){
	b2Assert(count >= 3);

	b2Vec2 center = new b2Vec2(0.0f, 0.0f);
	float32 area = 0.0f;
	float32 I = 0.0f;

	// pRef is the reference point for forming triangles.
	// It's location doesn't change the result (except for rounding error).
	b2Vec2 pRef = new b2Vec2(0.0f, 0.0f);
/*
#if 0
	// This code would put the reference point inside the polygon.
	for (int32 i = 0; i < count; ++i)
	{
		pRef += vs[i];
	}
	pRef *= 1.0f / count;
#endif
*/
	final float32 inv3 = 1.0f / 3.0f;

	for (int32 i = 0; i < count; ++i){
		// Triangle vertices.
		b2Vec2 p1 = pRef;
		b2Vec2 p2 = vs[i];
		b2Vec2 p3 = i + 1 < count ? vs[i+1] : vs[0];

		b2Vec2 e1 = b2Math.subtract(p2, p1);
		b2Vec2 e2 = b2Math.subtract(p3, p1);

		float32 D = b2Cross(e1, e2);

		float32 triangleArea = 0.5f * D;
		area += triangleArea;

		// Area weighted centroid
		center.add( b2Math.multiply(triangleArea * inv3, b2Math.add(p1 + b2Math.add(p2 + p3))));

		float32 px = p1.x, py = p1.y;
		float32 ex1 = e1.x, ey1 = e1.y;
		float32 ex2 = e2.x, ey2 = e2.y;

		float32 intx2 = inv3 * (0.25f * (ex1*ex1 + ex2*ex1 + ex2*ex2) + (px*ex1 + px*ex2)) + 0.5f*px*px;
		float32 inty2 = inv3 * (0.25f * (ey1*ey1 + ey2*ey1 + ey2*ey2) + (py*ey1 + py*ey2)) + 0.5f*py*py;

		I += D * (intx2 + inty2);
	}

	// Total mass
	massData.mass = rho * area;

	// Center of mass
	center.multiply( 1.0f / area );
	massData.center = center;

	// Inertia tensor relative to the center.
	I = rho * (I - area * b2Dot(center, center));
	massData.I = I;
}












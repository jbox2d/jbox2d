/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.gphysics.com
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
package org.jbox2d.collision;

import org.jbox2d.common.*;
import org.jbox2d.dynamics.Body;


//Updated to rev 56->108 of b2Shape.cpp/.h

public class CircleShape extends Shape {

    public float m_radius;
    public Vec2 m_localPosition;

    public CircleShape(ShapeDef def) {
    	super(def);
    	assert(def.type == ShapeType.CIRCLE_SHAPE);
    	CircleDef circleDef = (CircleDef)def;
    	m_type = ShapeType.CIRCLE_SHAPE;
    	m_localPosition = circleDef.localPosition.clone();
    	m_radius = circleDef.radius;
    }
    
    public void updateSweepRadius(Vec2 center) {
    	// Update the sweep radius (maximum radius) as measured from
    	// a local center point.
    	Vec2 d = m_localPosition.sub(center);
    	m_sweepRadius = d.length() + m_radius - Settings.toiSlop;
    }
    
    public boolean testPoint(XForm transform, Vec2 p) {
    	Vec2 center = transform.position.add(Mat22.mul(transform.R, m_localPosition));
    	Vec2 d = p.sub(center);
    	return Vec2.dot(d, d) <= m_radius * m_radius;
    }
    
    /*
 // Collision Detection in Interactive 3D Environments by Gino van den Bergen
 // From Section 3.1.2
 // x = s + a * r
 // norm(x) = radius
 bool b2CircleShape::TestSegment(const b2XForm& transform,
 								float32* lambda,
 								b2Vec2* normal,
 								const b2Segment& segment,
 								float32 maxLambda) const
 {
 	b2Vec2 position = transform.position + b2Mul(transform.R, m_localPosition);
 	b2Vec2 s = segment.p1 - position;
 	float32 b = b2Dot(s, s) - m_radius * m_radius;

 	// Does the segment start inside the circle?
 	if (b < 0.0f)
 	{
 		return false;
 	}

 	// Solve quadratic equation.
 	b2Vec2 r = segment.p2 - segment.p1;
 	float32 c =  b2Dot(s, r);
 	float32 rr = b2Dot(r, r);
 	float32 sigma = c * c - rr * b;

 	// Check for negative discriminant and short segment.
 	if (sigma < 0.0f || rr < FLT_EPSILON)
 	{
 		return false;
 	}

 	// Find the point of intersection of the line with the circle.
 	float32 a = -(c + sqrtf(sigma));

 	// Is the intersection point on the segment?
 	if (0.0f <= a && a <= maxLambda * rr)
 	{
 		a /= rr;
 		*lambda = a;
 		*normal = s + a * r;
 		normal->Normalize();
 		return true;
 	}

 	return false;
 }*/
    
    public void computeAABB(AABB aabb, XForm transform) {
    	Vec2 p = transform.position.add(Mat22.mul(transform.R, m_localPosition));
    	aabb.lowerBound.set(p.x - m_radius, p.y - m_radius);
    	aabb.upperBound.set(p.x + m_radius, p.y + m_radius);
    }

    public void computeSweptAABB(AABB aabb, XForm transform1, XForm transform2) {
    	Vec2 p1 = transform1.position.add(Mat22.mul(transform1.R, m_localPosition));
    	Vec2 p2 = transform2.position.add(Mat22.mul(transform2.R, m_localPosition));
    	Vec2 lower = Vec2.min(p1, p2);
    	Vec2 upper = Vec2.max(p1, p2);

    	aabb.lowerBound.set(lower.x - m_radius, lower.y - m_radius);
    	aabb.upperBound.set(upper.x + m_radius, upper.y + m_radius);
    }

    public void computeMass(MassData massData) {
    	massData.mass = m_density * (float)Math.PI * m_radius * m_radius;
    	massData.center = m_localPosition;

    	// inertia about the local origin
    	massData.I = massData.mass * (0.5f * m_radius * m_radius + Vec2.dot(m_localPosition, m_localPosition));
    }
    
    //I think all this has been moved to the Shape superclass
/*
    public void synchronize(Vec2 position1, Mat22 R1, Vec2 position2, Mat22 R2) {
        m_R.set(R2);
        m_position = m_R.mul(m_localPosition).addLocal(position2);

        if (m_proxyId == PairManager.NULL_PROXY) {
            return;
        }

        // Compute an AABB that covers the swept shape (may miss some rotation effect).
        Vec2 p1 = R1.mul(m_localPosition).addLocal(position1);
        Vec2 lower = Vec2.min(p1, m_position);
        Vec2 upper = Vec2.max(p1, m_position);

        AABB aabb = new AABB();
        aabb.lowerBound.set(lower.x - m_radius, lower.y - m_radius);
        aabb.upperBound.set(upper.x + m_radius, upper.y + m_radius);

        BroadPhase broadPhase = m_body.m_world.m_broadPhase;
        if (broadPhase.inRange(aabb)) {
            broadPhase.moveProxy(m_proxyId, aabb);
        } else {
            m_body.freeze();
        }
    }
    
    public void quickSync(Vec2 position, Mat22 R) {
        m_R = R.clone();
        m_position = R.mul(m_localPosition).addLocal(position); 
    }

    public Vec2 support(Vec2 d) {
        Vec2 u = d.clone();
        u.normalize();
        float r = Math.max(0.0f, m_radius - 2.0f * Settings.linearSlop);
        return u.mulLocal(r).addLocal(m_position); 
    }


    public void resetProxy(BroadPhase broadPhase) {
        if (m_proxyId == PairManager.NULL_PROXY) {
            return;
        }

        //Proxy proxy = broadPhase.getProxy(m_proxyId); //don't bother

        broadPhase.destroyProxy(m_proxyId);
        //proxy = null; //totally ineffective, but harmless...

        AABB aabb = new AABB();
        aabb.lowerBound.set(m_position.x - m_radius, m_position.y - m_radius);
        aabb.upperBound.set(m_position.x + m_radius, m_position.y + m_radius);

        if (broadPhase.inRange(aabb)) {
            m_proxyId = broadPhase.CreateProxy(aabb, this);
        } else {
            m_proxyId = PairManager.NULL_PROXY;
        }

        if (m_proxyId == PairManager.NULL_PROXY) {
            m_body.freeze();
        }
    }*/
    
    public float getRadius() {
    	return m_radius;
    }
    
    //Returns a copy of local position
    public Vec2 getLocalPosition() {
    	return m_localPosition.clone();
    }
}

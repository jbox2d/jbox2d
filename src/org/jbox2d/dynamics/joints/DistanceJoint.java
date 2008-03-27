/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.box2d.org
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

package org.jbox2d.dynamics.joints;

import org.jbox2d.common.*;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.dynamics.World;


//Updated to rev 56->130 of b2DistanceJoint.cpp/.h

//C = norm(p2 - p1) - L
//u = (p2 - p1) / norm(p2 - p1)
//Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//J = [-u -cross(r1, u) u cross(r2, u)]
//K = J * invM * JT
//= invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

/// A distance joint constrains two points on two bodies
/// to remain at a fixed distance from each other. You can view
/// this as a massless, rigid rod.

public class DistanceJoint extends Joint {
    
    public Vec2 m_localAnchor1;
    public Vec2 m_localAnchor2;
	public Vec2 m_u;
	public float m_force;
	public float m_mass;		// effective mass for the constraint.
	public float m_length;

    public DistanceJoint(DistanceJointDef def) {
        super(def);
        m_localAnchor1 = def.localAnchor1;
        m_localAnchor2 = def.localAnchor2;
        m_length = def.length;
        m_force = 0.0f;
        m_u = new Vec2();
    }

    @Override
    public Vec2 getAnchor1() {
    	return m_body1.getWorldPoint(m_localAnchor1);
    }

    @Override
    public Vec2 getAnchor2() {
        return m_body2.getWorldPoint(m_localAnchor2);
    }

    public Vec2 getReactionForce() {
    	return new Vec2(m_force * m_u.x, m_force * m_u.y);
    }

    public float getReactionTorque() {
        return 0.0f;
    }

    @Override
    public void initVelocityConstraints(TimeStep step) {
    	//TODO: fully inline temp Vec2 ops
    	Body b1 = m_body1;
    	Body b2 = m_body2;

    	// Compute the effective mass matrix.
    	Vec2 r1 = Mat22.mul(b1.m_xf.R, m_localAnchor1.sub(b1.getLocalCenter()));
    	Vec2 r2 = Mat22.mul(b2.m_xf.R, m_localAnchor2.sub(b2.getLocalCenter()));
    	m_u.x = b2.m_sweep.c.x + r2.x - b1.m_sweep.c.x - r1.x;
    	m_u.y = b2.m_sweep.c.y + r2.y - b1.m_sweep.c.y - r1.y;

    	// Handle singularity.
    	float length = m_u.length();
    	if (length > Settings.linearSlop) {
    		m_u.x *= 1.0f / length;
    		m_u.y *= 1.0f / length;
    	} else {
    		m_u.set(0.0f, 0.0f);
    	}

    	float cr1u = Vec2.cross(r1, m_u);
    	float cr2u = Vec2.cross(r2, m_u);
    	m_mass = b1.m_invMass + b1.m_invI * cr1u * cr1u + b2.m_invMass + b2.m_invI * cr2u * cr2u;
    	assert(m_mass > Settings.EPSILON);
    	m_mass = 1.0f / m_mass;

    	if (World.ENABLE_WARM_STARTING) {
    		float Px = step.dt * m_force * m_u.x;
    		float Py = step.dt * m_force * m_u.y;
    		b1.m_linearVelocity.x -= b1.m_invMass * Px;
    		b1.m_linearVelocity.y -= b1.m_invMass * Py;
    		b1.m_angularVelocity -= b1.m_invI * (r1.x*Py-r1.y*Px);//b2Cross(r1, P);
    		b2.m_linearVelocity.x += b2.m_invMass * Px;
    		b2.m_linearVelocity.y += b2.m_invMass * Py;
    		b2.m_angularVelocity += b2.m_invI * (r2.x*Py-r2.y*Px);//b2Cross(r2, P);
    	} else {
    		m_force = 0.0f;
    	}
    }

    @Override
    public boolean solvePositionConstraints() {
    	Body b1 = m_body1;
    	Body b2 = m_body2;

    	Vec2 r1 = Mat22.mul(b1.m_xf.R, m_localAnchor1.sub(b1.getLocalCenter()));
    	Vec2 r2 = Mat22.mul(b2.m_xf.R, m_localAnchor2.sub(b2.getLocalCenter()));

    	Vec2 d = new Vec2(b2.m_sweep.c.x + r2.x - b1.m_sweep.c.x - r1.x,
    					  b2.m_sweep.c.y + r2.y - b1.m_sweep.c.y - r1.y);

    	float length = d.normalize();
    	float C = length - m_length;
    	C = MathUtils.clamp(C, -Settings.maxLinearCorrection, Settings.maxLinearCorrection);

    	float impulse = -m_mass * C;
    	m_u = d;
    	float Px = impulse * m_u.x;
    	float Py = impulse * m_u.y;

    	b1.m_sweep.c.x -= b1.m_invMass * Px;
    	b1.m_sweep.c.y -= b1.m_invMass * Py;
    	b1.m_sweep.a -= b1.m_invI * (r1.x*Py-r1.y*Px);//b2Cross(r1, P);
    	b2.m_sweep.c.x += b2.m_invMass * Px;
    	b2.m_sweep.c.y += b2.m_invMass * Py;
    	b2.m_sweep.a += b2.m_invI * (r2.x*Py-r2.y*Px);//b2Cross(r2, P);

    	b1.synchronizeTransform();
    	b2.synchronizeTransform();

    	return Math.abs(C) < Settings.linearSlop;
    }

    @Override
    public void solveVelocityConstraints(TimeStep step) {
    	Body b1 = m_body1;
    	Body b2 = m_body2;

    	Vec2 r1 = Mat22.mul(b1.m_xf.R, m_localAnchor1.sub(b1.getLocalCenter()));
    	Vec2 r2 = Mat22.mul(b2.m_xf.R, m_localAnchor2.sub(b2.getLocalCenter()));

    	// Cdot = dot(u, v + cross(w, r))
    	Vec2 v1 = b1.m_linearVelocity.add(Vec2.cross(b1.m_angularVelocity, r1));
    	Vec2 v2 = b2.m_linearVelocity.add(Vec2.cross(b2.m_angularVelocity, r2));
    	float Cdot = Vec2.dot(m_u, v2.subLocal(v1));
    	float force = -step.inv_dt * m_mass * Cdot;
    	m_force += force;

    	float Px = step.dt * force * m_u.x;
    	float Py = step.dt * force * m_u.y;
    	b1.m_linearVelocity.x -= b1.m_invMass * Px;
    	b1.m_linearVelocity.y -= b1.m_invMass * Py;
    	b1.m_angularVelocity -= b1.m_invI * (r1.x*Py - r1.y*Px);//b2Cross(r1, P);
    	b2.m_linearVelocity.x += b2.m_invMass * Px;
    	b2.m_linearVelocity.y += b2.m_invMass * Py;
    	b2.m_angularVelocity += b2.m_invI * (r2.x*Py - r2.y*Px);//b2Cross(r2, P);
    }
}
/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DANIEL MURPHY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
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

import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.pooling.WorldPool;

//Updated to rev 56->130->142 of b2DistanceJoint.cpp/.h

//C = norm(p2 - p1) - L
//u = (p2 - p1) / norm(p2 - p1)
//Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//J = [-u -cross(r1, u) u cross(r2, u)]
//K = J * invM * JT
//= invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

/**
 * A distance joint constrains two points on two bodies
 * to remain at a fixed distance from each other. You can view
 * this as a massless, rigid rod.
 */
public class DistanceJoint extends Joint {
	
	public final Vec2 m_localAnchor1;
	public final Vec2 m_localAnchor2;
	public final Vec2 m_u;
	public float m_impulse;
	public float m_mass; // effective mass for the constraint.
	public float m_length;
	public float m_frequencyHz;
	public float m_dampingRatio;
	public float m_gamma;
	public float m_bias;
	
	public DistanceJoint(WorldPool argWorld, final DistanceJointDef def) {
		super(argWorld, def);
		m_localAnchor1 = def.localAnchorA.clone();
		m_localAnchor2 = def.localAnchorB.clone();
		m_length = def.length;
		m_impulse = 0.0f;
		m_u = new Vec2();
		m_frequencyHz = def.frequencyHz;
		m_dampingRatio = def.dampingRatio;
		m_gamma = 0.0f;
		m_bias = 0.0f;
	}
	
	public void setFrequency(float hz) {
		m_frequencyHz = hz;
	}
	
	public float getFrequency() {
		return m_frequencyHz;
	}
	
	public float getLength() {
		return m_length;
	}
	
	public void setLength(float argLength) {
		m_length = argLength;
	}
	
	public void setDampingRatio(float damp) {
		m_dampingRatio = damp;
	}
	
	public float getDampingRatio() {
		return m_dampingRatio;
	}
	
	@Override
	public void getAnchorA(Vec2 argOut) {
		m_bodyA.getWorldPointToOut(m_localAnchor1, argOut);
	}
	
	@Override
	public void getAnchorB(Vec2 argOut) {
		m_bodyB.getWorldPointToOut(m_localAnchor2, argOut);
	}
	
	// djm pooled
	@Override
	public void getReactionForce(float inv_dt, Vec2 argOut) {
		argOut.x = m_impulse * m_u.x * inv_dt;
		argOut.y = m_impulse * m_u.y * inv_dt;
	}
	
	@Override
	public float getReactionTorque(float inv_dt) {
		return 0.0f;
	}
	
	@Override
	public void initVelocityConstraints(final TimeStep step) {
		
		// TODO: fully inline temp Vec2 ops
		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;
		
		Vec2 r1 = pool.popVec2();
		Vec2 r2 = pool.popVec2();
		
		// Compute the effective mass matrix.
		r1.set(m_localAnchor1).subLocal(b1.getLocalCenter());
		r2.set(m_localAnchor2).subLocal(b2.getLocalCenter());
		Mat22.mulToOut(b1.getTransform().R, r1, r1);
		Mat22.mulToOut(b2.getTransform().R, r2, r2);
		
		m_u.x = b2.m_sweep.c.x + r2.x - b1.m_sweep.c.x - r1.x;
		m_u.y = b2.m_sweep.c.y + r2.y - b1.m_sweep.c.y - r1.y;
		
		// Handle singularity.
		float length = m_u.length();
		if (length > Settings.linearSlop) {
			m_u.x *= 1.0f / length;
			m_u.y *= 1.0f / length;
		}
		else {
			m_u.set(0.0f, 0.0f);
		}
		
		float cr1u = Vec2.cross(r1, m_u);
		float cr2u = Vec2.cross(r2, m_u);
		
		float invMass = b1.m_invMass + b1.m_invI * cr1u * cr1u + b2.m_invMass + b2.m_invI * cr2u * cr2u;
		assert (invMass > Settings.EPSILON);
		m_mass = 1.0f / invMass;
		
		if (m_frequencyHz > 0.0f) {
			float C = length - m_length;
			
			// Frequency
			float omega = 2.0f * MathUtils.PI * m_frequencyHz;
			
			// Damping coefficient
			float d = 2.0f * m_mass * m_dampingRatio * omega;
			
			// Spring stiffness
			float k = m_mass * omega * omega;
			
			// magic formulas
			m_gamma = step.dt * (d + step.dt * k);
			m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
			m_bias = C * step.dt * k * m_gamma;
			
			m_mass = invMass + m_gamma;
			m_mass = m_mass != 0.0f ? 1.0f / m_mass : 0.0f;
		}
		
		if (step.warmStarting) {
			
			// Scale the impulse to support a variable time step.
			m_impulse *= step.dtRatio;
			
			Vec2 P = pool.popVec2();
			P.set(m_u).mulLocal(m_impulse);
			
			b1.m_linearVelocity.x -= b1.m_invMass * P.x;
			b1.m_linearVelocity.y -= b1.m_invMass * P.y;
			b1.m_angularVelocity -= b1.m_invI * Vec2.cross(r1, P);
			
			b2.m_linearVelocity.x += b2.m_invMass * P.x;
			b2.m_linearVelocity.y += b2.m_invMass * P.y;
			b2.m_angularVelocity += b2.m_invI * Vec2.cross(r2, P);
			
			pool.pushVec2(1);
		}
		else {
			m_impulse = 0.0f;
		}
		pool.pushVec2(2);
	}
	
	@Override
	public void solveVelocityConstraints(final TimeStep step) {
		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;
		
		final Vec2 r1 = pool.popVec2();
		final Vec2 r2 = pool.popVec2();
		
		r1.set(m_localAnchor1).subLocal(b1.getLocalCenter());
		r2.set(m_localAnchor2).subLocal(b2.getLocalCenter());
		Mat22.mulToOut(b1.getTransform().R, r1, r1);
		Mat22.mulToOut(b2.getTransform().R, r2, r2);
		
		final Vec2 v1 = pool.popVec2();
		final Vec2 v2 = pool.popVec2();
		
		// Cdot = dot(u, v + cross(w, r))
		Vec2.crossToOut(b1.m_angularVelocity, r1, v1);
		Vec2.crossToOut(b2.m_angularVelocity, r2, v2);
		v1.set(b1.m_linearVelocity).addLocal(b1.m_linearVelocity);
		v2.set(b2.m_linearVelocity).addLocal(b2.m_linearVelocity);
		
		float Cdot = Vec2.dot(m_u, v2.subLocal(v1));
		
		float impulse = -m_mass * (Cdot + m_bias + m_gamma * m_impulse);
		m_impulse += impulse;
		
		float Px = impulse * m_u.x;
		float Py = impulse * m_u.y;
		b1.m_linearVelocity.x -= b1.m_invMass * Px;
		b1.m_linearVelocity.y -= b1.m_invMass * Py;
		b1.m_angularVelocity -= b1.m_invI * (r1.x * Py - r1.y * Px);// b2Cross(r1, P);
		b2.m_linearVelocity.x += b2.m_invMass * Px;
		b2.m_linearVelocity.y += b2.m_invMass * Py;
		b2.m_angularVelocity += b2.m_invI * (r2.x * Py - r2.y * Px);// b2Cross(r2, P);
		
		pool.pushVec2(4);
	}
	
	@Override
	public boolean solvePositionConstraints(float baumgarte) {
		if (m_frequencyHz > 0.0f) {
			return true;
		}
		
		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;
		
		final Vec2 r1 = pool.popVec2();
		final Vec2 r2 = pool.popVec2();
		final Vec2 d = pool.popVec2();
		
		r1.set(m_localAnchor1).subLocal(b1.getLocalCenter());
		r2.set(m_localAnchor2).subLocal(b2.getLocalCenter());
		Mat22.mulToOut(b1.getTransform().R, r1, r1);
		Mat22.mulToOut(b2.getTransform().R, r2, r2);
		
		d.x = b2.m_sweep.c.x + r2.x - b1.m_sweep.c.x - r1.x;
		d.y = b2.m_sweep.c.y + r2.y - b1.m_sweep.c.y - r1.y;
		
		float length = d.normalize();
		float C = length - m_length;
		C = MathUtils.clamp(C, -Settings.maxLinearCorrection, Settings.maxLinearCorrection);
		
		float impulse = -m_mass * C;
		m_u.set(d);
		float Px = impulse * m_u.x;
		float Py = impulse * m_u.y;
		
		b1.m_sweep.c.x -= b1.m_invMass * Px;
		b1.m_sweep.c.y -= b1.m_invMass * Py;
		b1.m_sweep.a -= b1.m_invI * (r1.x * Py - r1.y * Px);// b2Cross(r1, P);
		
		b2.m_sweep.c.x += b2.m_invMass * Px;
		b2.m_sweep.c.y += b2.m_invMass * Py;
		b2.m_sweep.a += b2.m_invI * (r2.x * Py - r2.y * Px);// b2Cross(r2, P);
		
		b1.synchronizeTransform();
		b2.synchronizeTransform();
		
		pool.pushVec2(3);
		
		return MathUtils.abs(C) < Settings.linearSlop;
	}
}

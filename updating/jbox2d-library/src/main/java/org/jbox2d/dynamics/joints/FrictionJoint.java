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
/**
 * Created at 7:27:32 AM Jan 20, 2011
 */
package org.jbox2d.dynamics.joints;

import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.pooling.WorldPool;

/**
 * @author Daniel Murphy
 */
public class FrictionJoint extends Joint {
	
	private final Vec2 m_localAnchorA;
	private final Vec2 m_localAnchorB;
	
	private final Mat22 m_linearMass;
	private float m_angularMass;
	
	private final Vec2 m_linearImpulse;
	private float m_angularImpulse;
	
	private float m_maxForce;
	private float m_maxTorque;
	
	/**
	 * @param argWorldPool
	 * @param def
	 */
	public FrictionJoint(WorldPool argWorldPool, FrictionJointDef def) {
		super(argWorldPool, def);
		m_localAnchorA = new Vec2(def.localAnchorA);
		m_localAnchorB = new Vec2(def.localAnchorB);
		
		m_linearImpulse = new Vec2();
		m_angularImpulse = 0.0f;
		
		m_maxForce = def.maxForce;
		m_maxTorque = def.maxTorque;
		
		m_linearMass = new Mat22();
	}
	
	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getAnchorA(org.jbox2d.common.Vec2)
	 */
	@Override
	public void getAnchorA(Vec2 argOut) {
		m_bodyA.getWorldPointToOut(m_localAnchorA, argOut);
	}
	
	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getAnchorB(org.jbox2d.common.Vec2)
	 */
	@Override
	public void getAnchorB(Vec2 argOut) {
		m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
	}
	
	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getReactionForce(float,
	 *      org.jbox2d.common.Vec2)
	 */
	@Override
	public void getReactionForce(float inv_dt, Vec2 argOut) {
		argOut.set(m_linearImpulse).mulLocal(inv_dt);
	}
	
	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getReactionTorque(float)
	 */
	@Override
	public float getReactionTorque(float inv_dt) {
		return inv_dt * m_angularImpulse;
	}
	
	public void setMaxForce(float force) {
		assert (force >= 0.0f);
		m_maxForce = force;
	}
	
	public float getMaxForce() {
		return m_maxForce;
	}
	
	public void setMaxTorque(float torque) {
		assert (torque >= 0.0f);
		m_maxTorque = torque;
	}
	
	public float getMaxTorque() {
		return m_maxTorque;
	}
	
	/**
	 * @see org.jbox2d.dynamics.joints.Joint#initVelocityConstraints(org.jbox2d.dynamics.TimeStep)
	 */
	@Override
	public void initVelocityConstraints(TimeStep step) {
		Body bA = m_bodyA;
		Body bB = m_bodyB;
		
		// Compute the effective mass matrix.
		final Vec2 rA = pool.popVec2();
		final Vec2 rB = pool.popVec2();
		
		rA.set(m_localAnchorA).subLocal(bA.getLocalCenter());
		rB.set(m_localAnchorB).subLocal(bB.getLocalCenter());
		Mat22.mulToOut(bA.getTransform().R, rA, rA);
		Mat22.mulToOut(bB.getTransform().R, rB, rB);
		
		// J = [-I -r1_skew I r2_skew]
		// [ 0 -1 0 1]
		// r_skew = [-ry; rx]
		
		// Matlab
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB, -r1y*iA*r1x-r2y*iB*r2x, -r1y*iA-r2y*iB]
		// [ -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB, r1x*iA+r2x*iB]
		// [ -r1y*iA-r2y*iB, r1x*iA+r2x*iB, iA+iB]
		
		float mA = bA.m_invMass, mB = bB.m_invMass;
		float iA = bA.m_invI, iB = bB.m_invI;
		
		final Mat22 K1 = pool.popMat22();
		K1.col1.x = mA + mB;
		K1.col2.x = 0.0f;
		K1.col1.y = 0.0f;
		K1.col2.y = mA + mB;
		
		final Mat22 K2 = pool.popMat22();
		K2.col1.x = iA * rA.y * rA.y;
		K2.col2.x = -iA * rA.x * rA.y;
		K2.col1.y = -iA * rA.x * rA.y;
		K2.col2.y = iA * rA.x * rA.x;
		
		final Mat22 K3 = pool.popMat22();
		K3.col1.x = iB * rB.y * rB.y;
		K3.col2.x = -iB * rB.x * rB.y;
		K3.col1.y = -iB * rB.x * rB.y;
		K3.col2.y = iB * rB.x * rB.x;
		
		K1.addLocal(K2).addLocal(K3);
		m_linearMass.set(K1).invertLocal();
		
		m_angularMass = iA + iB;
		if (m_angularMass > 0.0f) {
			m_angularMass = 1.0f / m_angularMass;
		}
		
		if (step.warmStarting) {
			// Scale impulses to support a variable time step.
			m_linearImpulse.mulLocal(step.dtRatio);
			m_angularImpulse *= step.dtRatio;
			
			final Vec2 P = pool.popVec2();
			P.set(m_linearImpulse.x, m_linearImpulse.y);
			
			final Vec2 temp = pool.popVec2();
			temp.set(P).mulLocal(mA);
			bA.m_linearVelocity.subLocal(temp);
			bA.m_angularVelocity -= iA * (Vec2.cross(rA, P) + m_angularImpulse);
			
			temp.set(P).mulLocal(mB);
			bB.m_linearVelocity.addLocal(temp);
			bB.m_angularVelocity += iB * (Vec2.cross(rB, P) + m_angularImpulse);
			
			pool.pushVec2(2);
		}
		else {
			m_linearImpulse.setZero();
			m_angularImpulse = 0.0f;
		}
		
		pool.pushVec2(2);
		pool.pushMat22(3);
	}
	
	/**
	 * @see org.jbox2d.dynamics.joints.Joint#solveVelocityConstraints(org.jbox2d.dynamics.TimeStep)
	 */
	@Override
	public void solveVelocityConstraints(TimeStep step) {
		Body bA = m_bodyA;
		Body bB = m_bodyB;
		
		final Vec2 vA = bA.m_linearVelocity;
		float wA = bA.m_angularVelocity;
		final Vec2 vB = bB.m_linearVelocity;
		float wB = bB.m_angularVelocity;
		
		float mA = bA.m_invMass, mB = bB.m_invMass;
		float iA = bA.m_invI, iB = bB.m_invI;
		
		final Vec2 rA = pool.popVec2();
		final Vec2 rB = pool.popVec2();
		
		rA.set(m_localAnchorA).subLocal(bA.getLocalCenter());
		rB.set(m_localAnchorB).subLocal(bB.getLocalCenter());
		Mat22.mulToOut(bA.getTransform().R, rA, rA);
		Mat22.mulToOut(bB.getTransform().R, rB, rB);
		
		// Solve angular friction
		{
			float Cdot = wB - wA;
			float impulse = -m_angularMass * Cdot;
			
			float oldImpulse = m_angularImpulse;
			float maxImpulse = step.dt * m_maxTorque;
			m_angularImpulse = MathUtils.clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = m_angularImpulse - oldImpulse;
			
			wA -= iA * impulse;
			wB += iB * impulse;
		}
		
		// Solve linear friction
		{
			final Vec2 Cdot = pool.popVec2();
			final Vec2 temp = pool.popVec2();
			
			Vec2.crossToOut(wA, rA, temp);
			Vec2.crossToOut(wB, rB, Cdot);
			Cdot.addLocal(vB).subLocal(vA).subLocal(temp);
			
			final Vec2 impulse = pool.popVec2();
			Mat22.mulToOut(m_linearMass, Cdot, impulse);
			impulse.negateLocal();
			
			
			final Vec2 oldImpulse = pool.popVec2();
			oldImpulse.set(m_linearImpulse);
			m_linearImpulse.addLocal(impulse);
			
			float maxImpulse = step.dt * m_maxForce;
			
			if (m_linearImpulse.lengthSquared() > maxImpulse * maxImpulse) {
				m_linearImpulse.normalize();
				m_linearImpulse.mulLocal(maxImpulse);
			}
			
			impulse.set(m_linearImpulse).subLocal(oldImpulse);
			
			temp.set(impulse).mulLocal(mA);
			vA.subLocal(temp);
			wA -= iA * Vec2.cross(rA, impulse);
			
			temp.set(impulse).mulLocal(mB);
			vB.addLocal(temp);
			wB += iB * Vec2.cross(rB, impulse);
		}
		
		pool.pushVec2(6);
		
		bA.m_angularVelocity = wA;
		bB.m_angularVelocity = wB;
	}
	
	/**
	 * @see org.jbox2d.dynamics.joints.Joint#solvePositionConstraints(float)
	 */
	@Override
	public boolean solvePositionConstraints(float baumgarte) {
		return true;
	}
}

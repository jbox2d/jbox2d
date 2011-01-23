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
 * Created at 3:38:38 AM Jan 15, 2011
 */
package org.jbox2d.dynamics.joints;

import org.jbox2d.common.Mat22;
import org.jbox2d.common.Mat33;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.Vec3;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.pooling.WorldPool;

/**
 * @author Daniel Murphy
 */
public class WeldJoint extends Joint {
	
	private final Vec2 m_localAnchorA;
	private final Vec2 m_localAnchorB;
	private float m_referenceAngle;

	private final Vec3 m_impulse;

	private final Mat33 m_mass;
	
	/**
	 * @param argWorld
	 * @param def
	 */
	protected WeldJoint(WorldPool argWorld, WeldJointDef def) {
		super(argWorld, def);
		m_localAnchorA = new Vec2(def.localAnchorA);
		m_localAnchorB = new Vec2(def.localAnchorB);
		m_referenceAngle = def.referenceAngle;

		m_impulse = new Vec3();
		m_impulse.setZero();
		
		m_mass = new Mat33();
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
	 * @see org.jbox2d.dynamics.joints.Joint#getReactionForce(float, org.jbox2d.common.Vec2)
	 */
	@Override
	public void getReactionForce(float inv_dt, Vec2 argOut) {
		argOut.set(m_impulse.x, m_impulse.y);
		argOut.mulLocal(inv_dt);
	}
	
	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getReactionTorque(float)
	 */
	@Override
	public float getReactionTorque(float inv_dt) {
		return inv_dt * m_impulse.z;
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
		//     [ 0       -1 0       1]
		// r_skew = [-ry; rx]

		// Matlab
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
		//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

		float mA = bA.m_invMass, mB = bB.m_invMass;
		float iA = bA.m_invI, iB = bB.m_invI;

		m_mass.col1.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		m_mass.col2.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		m_mass.col3.x = -rA.y * iA - rB.y * iB;
		m_mass.col1.y = m_mass.col2.x;
		m_mass.col2.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		m_mass.col3.y = rA.x * iA + rB.x * iB;
		m_mass.col1.z = m_mass.col3.x;
		m_mass.col2.z = m_mass.col3.y;
		m_mass.col3.z = iA + iB;

		if (step.warmStarting)
		{
			// Scale impulses to support a variable time step.
			m_impulse.mulLocal(step.dtRatio);
			
			final Vec2 P = pool.popVec2();
			final Vec2 temp = pool.popVec2();
			P.set(m_impulse.x, m_impulse.y);

			temp.set(P).mulLocal(mA);
			bA.m_linearVelocity.subLocal(temp);
			bA.m_angularVelocity -= iA * (Vec2.cross(rA, P) + m_impulse.z);

			temp.set(P).mulLocal(mB);
			bB.m_linearVelocity.addLocal(temp);
			bB.m_angularVelocity += iB * (Vec2.cross(rB, P) + m_impulse.z);
			
			pool.pushVec2(2);
		}
		else
		{
			m_impulse.setZero();
		}
		pool.pushVec2(2);
	}
	
	/**
	 * @see org.jbox2d.dynamics.joints.Joint#solveVelocityConstraints(org.jbox2d.dynamics.TimeStep)
	 */
	@Override
	public void solveVelocityConstraints(TimeStep step) {
		Body bA = m_bodyA;
		Body bB = m_bodyB;

		Vec2 vA = bA.m_linearVelocity;
		float wA = bA.m_angularVelocity;
		Vec2 vB = bB.m_linearVelocity;
		float wB = bB.m_angularVelocity;

		float mA = bA.m_invMass, mB = bB.m_invMass;
		float iA = bA.m_invI, iB = bB.m_invI;
		
		final Vec2 rA = pool.popVec2();
		final Vec2 rB = pool.popVec2();
		rA.set(m_localAnchorA).subLocal(bA.getLocalCenter());
		rB.set(m_localAnchorB).subLocal(bB.getLocalCenter());
		Mat22.mulToOut(bA.getTransform().R, rA, rA);
		Mat22.mulToOut(bB.getTransform().R, rB, rB);
		
		
		final Vec2 Cdot1 = pool.popVec2();
		final Vec2 temp = pool.popVec2();
		// Solve point-to-point finalraint
		Vec2.crossToOut(wA, rA, temp);
		Vec2.crossToOut(wB, rB, Cdot1);
		Cdot1.addLocal(vB).subLocal(vA).subLocal(temp);
		float Cdot2 = wB - wA;
		
		final Vec3 Cdot = pool.popVec3();
		Cdot.set(Cdot1.x, Cdot1.y, Cdot2);

		final Vec3 impulse = pool.popVec3();
		m_mass.solve33ToOut(Cdot.negateLocal(), impulse); // just leave Cdot negated..
		m_impulse.addLocal(impulse);

		final Vec2 P = pool.popVec2();
		P.set(impulse.x, impulse.y);

		temp.set(P).mulLocal(mA);
		vA.subLocal(temp);
		wA -= iA * (Vec2.cross(rA, P) + impulse.z);
		
		temp.set(P).mulLocal(mB);
		vB.addLocal(temp);
		wB += iB * (Vec2.cross(rB, P) + impulse.z);

		bA.m_linearVelocity.set(vA);
		bA.m_angularVelocity = wA;
		bB.m_linearVelocity.set(vB);
		bB.m_angularVelocity = wB;
		
		pool.pushVec2(5);
		pool.pushVec3(2);
	}
	
	/**
	 * @see org.jbox2d.dynamics.joints.Joint#solvePositionConstraints(float)
	 */
	@Override
	public boolean solvePositionConstraints(float baumgarte) {
		Body bA = m_bodyA;
		Body bB = m_bodyB;

		float mA = bA.m_invMass, mB = bB.m_invMass;
		float iA = bA.m_invI, iB = bB.m_invI;

		final Vec2 rA = pool.popVec2();
		final Vec2 rB = pool.popVec2();
		rA.set(m_localAnchorA).subLocal(bA.getLocalCenter());
		rB.set(m_localAnchorB).subLocal(bB.getLocalCenter());
		Mat22.mulToOut(bA.getTransform().R, rA, rA);
		Mat22.mulToOut(bB.getTransform().R, rB, rB);
		
		final Vec2 C1 = pool.popVec2();
		C1.set(bB.m_sweep.c).addLocal(rB).subLocal(bA.m_sweep.c).subLocal(rA);
		float C2 = bB.m_sweep.a - bA.m_sweep.a - m_referenceAngle;

		// Handle large detachment.
		final float k_allowedStretch = 10.0f * Settings.linearSlop;
		float positionError = C1.length();
		float angularError = MathUtils.abs(C2);
		if (positionError > k_allowedStretch){
			iA *= 1.0f;
			iB *= 1.0f;
		}

		m_mass.col1.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		m_mass.col2.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		m_mass.col3.x = -rA.y * iA - rB.y * iB;
		m_mass.col1.y = m_mass.col2.x;
		m_mass.col2.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		m_mass.col3.y = rA.x * iA + rB.x * iB;
		m_mass.col1.z = m_mass.col3.x;
		m_mass.col2.z = m_mass.col3.y;
		m_mass.col3.z = iA + iB;

		final Vec3 C = pool.popVec3();
		final Vec3 impulse = pool.popVec3();
		C.set(C1.x, C1.y, C2);

		m_mass.solve33ToOut(C.negateLocal(), impulse); // just leave c negated..

		final Vec2 P = pool.popVec2();
		final Vec2 temp = pool.popVec2();
		P.set(impulse.x, impulse.y);

		temp.set(P).mulLocal(mA);
		bA.m_sweep.c.subLocal(temp);
		bA.m_sweep.a -= iA * (Vec2.cross(rA, P) + impulse.z);

		temp.set(P).mulLocal(mB);
		bB.m_sweep.c.addLocal(temp);
		bB.m_sweep.a += iB * (Vec2.cross(rB, P) + impulse.z);

		bA.synchronizeTransform();
		bB.synchronizeTransform();
		
		pool.pushVec2(5);
		pool.pushVec3(2);

		return positionError <= Settings.linearSlop && angularError <= Settings.angularSlop;
	}
	
}

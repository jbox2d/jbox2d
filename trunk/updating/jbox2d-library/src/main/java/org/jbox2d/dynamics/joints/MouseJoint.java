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
package org.jbox2d.dynamics.joints;

import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.pooling.WorldPool;

public class MouseJoint extends Joint {

	private final Vec2 m_localAnchor = new Vec2();
	private final Vec2 m_target = new Vec2();
	private final Vec2 m_impulse = new Vec2();

	private final Mat22 m_mass = new Mat22();	// effective mass for point-to-point constraint.
	private final Vec2 m_C = new Vec2();		// position error
	private float m_maxForce;
	private float m_frequencyHz;
	private float m_dampingRatio;
	private float m_beta;
	private float m_gamma;
	

	protected MouseJoint(WorldPool argWorld, MouseJointDef def) {
		super(argWorld, def);
		assert(def.target.isValid());
		assert(def.maxForce >= 0);
		assert(def.frequencyHz >= 0);
		assert(def.dampingRatio >= 0);
		
		m_target.set(def.target);
		Transform.mulTransToOut(m_bodyB.getTransform(), m_target, m_localAnchor);
		
		m_maxForce = def.maxForce;
		m_impulse.setZero();
		
		m_frequencyHz = def.frequencyHz;
		m_dampingRatio = def.dampingRatio;
		
		m_beta = 0;
		m_gamma = 0;
	}
	
	@Override
	public void getAnchorA(Vec2 argOut) {
		argOut.set(m_target);
	}

	@Override
	public void getAnchorB(Vec2 argOut) {
		m_bodyB.getWorldPointToOut(m_localAnchor, argOut);
	}

	@Override
	public void getReactionForce(float invDt, Vec2 argOut) {
		argOut.set(m_impulse).mulLocal(invDt);
	}

	@Override
	public float getReactionTorque(float invDt) {
		return invDt * 0.0f;
	}

	
	public void setTarget( Vec2 target){
		if(m_bodyB.isAwake() == false){
			m_bodyB.setAwake(true);
		}
		m_target.set(target);
	}
	public Vec2 getTarget(){
		return m_target;
	}

	/// set/get the maximum force in Newtons.
	public void setMaxForce(float force){
		m_maxForce = force;
	}
	public float getMaxForce(){
		return m_maxForce;
	}

	/// set/get the frequency in Hertz.
	public void setFrequency(float hz){
		m_frequencyHz = hz;
	}
	public float getFrequency(){
		return m_frequencyHz;
	}

	/// set/get the damping ratio (dimensionless).
	public void setDampingRatio(float ratio){
		m_dampingRatio = ratio;
	}
	public float getDampingRatio(){
		return m_dampingRatio;
	}

	@Override
	public void initVelocityConstraints(TimeStep step) {
		Body b = m_bodyB;

		float mass = b.getMass();

		// Frequency
		float omega = 2.0f * MathUtils.PI * m_frequencyHz;

		// Damping coefficient
		float d = 2.0f * mass * m_dampingRatio * omega;

		// Spring stiffness
		float k = mass * (omega * omega);

		// magic formulas
		// gamma has units of inverse mass.
		// beta has units of inverse time.
		assert(d + step.dt * k > Settings.EPSILON);
		m_gamma = step.dt * (d + step.dt * k);
		if (m_gamma != 0.0f){
			m_gamma = 1.0f / m_gamma;
		}
		m_beta = step.dt * k * m_gamma;

		Vec2 r = pool.popVec2();
		
		// Compute the effective mass matrix.
		//Vec2 r = Mul(b.getTransform().R, m_localAnchor - b.getLocalCenter());
		r.set(m_localAnchor).subLocal(b.getLocalCenter());
		Mat22.mulToOut(b.getTransform().R, r, r);
		
		// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
		//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
		//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
		float invMass = b.m_invMass;
		float invI = b.m_invI;

		Mat22 K1 = pool.popMat22();
		K1.col1.x = invMass;	K1.col2.x = 0.0f;
		K1.col1.y = 0.0f;		K1.col2.y = invMass;

		Mat22 K2 = pool.popMat22();
		K2.col1.x =  invI * r.y * r.y;	K2.col2.x = -invI * r.x * r.y;
		K2.col1.y = -invI * r.x * r.y;	K2.col2.y =  invI * r.x * r.x;

		Mat22 K = pool.popMat22();
		K.set(K1).addLocal(K2);
		K.col1.x += m_gamma;
		K.col2.y += m_gamma;

		K.invertToOut(m_mass);

		m_C.set(b.m_sweep.c).addLocal(r).subLocal(m_target);

		// Cheat with some damping
		b.m_angularVelocity *= 0.98f;

		// Warm starting.
		m_impulse.mulLocal(step.dtRatio);
		// pool
		Vec2 temp = pool.popVec2();
		temp.set(m_impulse).mulLocal(invMass);
		b.m_linearVelocity.addLocal(temp);
		b.m_angularVelocity += invI * Vec2.cross(r, m_impulse);
		
		pool.pushVec2(2);
		pool.pushMat22(3);
	}

	@Override
	public boolean solvePositionConstraints(float baumgarte) {
		return true;
	}

	@Override
	public void solveVelocityConstraints(TimeStep step) {
		Body b = m_bodyB;

		Vec2 r = pool.popVec2();

		r.set(m_localAnchor).subLocal(b.getLocalCenter());
		Mat22.mulToOut(b.getTransform().R, r, r);
		
		// Cdot = v + cross(w, r)
		Vec2 Cdot = pool.popVec2();
		Vec2.crossToOut(b.m_angularVelocity, r, Cdot);
		Cdot.addLocal(b.m_linearVelocity);
		
		Vec2 impulse = pool.popVec2();
		Vec2 temp = pool.popVec2();
		
		//Mul(m_mass, -(Cdot + m_beta * m_C + m_gamma * m_impulse));
		impulse.set(m_C).mulLocal(m_beta);
		temp.set(m_impulse).mulLocal(m_gamma);
		temp.addLocal(impulse).addLocal(Cdot).mulLocal(-1);
		Mat22.mulToOut(m_mass, temp, impulse);

		Vec2 oldImpulse = temp;
		oldImpulse.set(m_impulse);
		m_impulse.addLocal(impulse);
		float maxImpulse = step.dt * m_maxForce;
		if (m_impulse.lengthSquared() > maxImpulse * maxImpulse){
			m_impulse.mulLocal(maxImpulse / m_impulse.length());
		}
		impulse.set(m_impulse).subLocal(oldImpulse);

		// pooling
		oldImpulse.set(impulse).mulLocal(b.m_invMass);
		b.m_linearVelocity.addLocal(oldImpulse);
		b.m_angularVelocity += b.m_invI * Vec2.cross(r, impulse);
		
		pool.pushVec2(4);
	}

}

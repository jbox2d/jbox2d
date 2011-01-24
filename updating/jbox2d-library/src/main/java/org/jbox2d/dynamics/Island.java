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
package org.jbox2d.dynamics;

import org.jbox2d.callbacks.ContactImpulse;
import org.jbox2d.callbacks.ContactListener;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.contacts.ContactConstraint;
import org.jbox2d.dynamics.contacts.ContactSolver;
import org.jbox2d.dynamics.joints.Joint;

/*
Position Correction Notes
=========================
I tried the several algorithms for position correction of the 2D revolute joint.
I looked at these systems:
- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
- suspension bridge with 30 1m long planks of length 1m.
- multi-link chain with 30 1m long links.

Here are the algorithms:

Baumgarte - A fraction of the position error is added to the velocity error. There is no
separate position solver.

Pseudo Velocities - After the velocity solver and position integration,
the position error, Jacobian, and effective mass are recomputed. Then
the velocity constraints are solved with pseudo velocities and a fraction
of the position error is added to the pseudo velocity error. The pseudo
velocities are initialized to zero and there is no warm-starting. After
the position solver, the pseudo velocities are added to the positions.
This is also called the First Order World method or the Position LCP method.

Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
position error is re-computed for each raint and the positions are updated
after the raint is solved. The radius vectors (aka Jacobians) are
re-computed too (otherwise the algorithm has horrible instability). The pseudo
velocity states are not needed because they are effectively zero at the beginning
of each iteration. Since we have the current position error, we allow the
iterations to terminate early if the error becomes smaller than _linearSlop.

Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
each time a raint is solved.

Here are the results:
Baumgarte - this is the cheapest algorithm but it has some stability problems,
especially with the bridge. The chain links separate easily close to the root
and they jitter as they struggle to pull together. This is one of the most common
methods in the field. The big drawback is that the position correction artificially
affects the momentum, thus leading to instabilities and false bounce. I used a
bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
factor makes joints and contacts more spongy.

Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
stable. However, joints still separate with large angular velocities. Drag the
simple pendulum in a circle quickly and the joint will separate. The chain separates
easily and does not recover. I used a bias factor of 0.2. A larger value lead to
the bridge collapsing when a heavy cube drops on it.

Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
Velocities, but in other ways it is worse. The bridge and chain are much more
stable, but the simple pendulum goes unstable at high angular velocities.

Full NGS - stable in all tests. The joints display good stiffness. The bridge
still sags, but this is better than infinite forces.

Recommendations
Pseudo Velocities are not really worthwhile because the bridge and chain cannot
recover from joint separation. In other cases the benefit over Baumgarte is small.

Modified NGS is not a robust method for the revolute joint due to the violent
instability seen in the simple pendulum. Perhaps it is viable with other raint
types, especially scalar constraints where the effective mass is a scalar.

This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
and is very fast. I don't think we can escape Baumgarte, especially in highly
demanding cases where high raint fidelity is not needed.

Full NGS is robust and easy on the eyes. I recommend this as an option for
higher fidelity simulation and certainly for suspension bridges and long chains.
Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
joint separation can be problematic. The number of NGS iterations can be reduced
for better performance without harming robustness much.

Each joint in a can be handled differently in the position solver. So I recommend
a system where the user can select the algorithm on a per joint basis. I would
probably default to the slower Full NGS and let the user select the faster
Baumgarte method in performance critical scenarios.
*/

/*
Cache Performance

The Box2D solvers are dominated by cache misses. Data structures are designed
to increase the number of cache hits. Much of misses are due to random access
to body data. The raint structures are iterated over linearly, which leads
to few cache misses.

The bodies are not accessed during iteration. Instead read only data, such as
the mass values are stored with the constraints. The mutable data are the raint
impulses and the bodies velocities/positions. The impulses are held inside the
raint structures. The body velocities/positions are held in compact, temporary
arrays to increase the number of cache hits. Linear and angular velocity are
stored in a single array since multiple arrays lead to multiple misses.
*/

/*
2D Rotation

R = [cos(theta) -sin(theta)]
    [sin(theta) cos(theta) ]

thetaDot = omega

Let q1 = cos(theta), q2 = sin(theta).
R = [q1 -q2]
    [q2  q1]

q1Dot = -thetaDot * q2
q2Dot = thetaDot * q1

q1_new = q1_old - dt * w * q2
q2_new = q2_old + dt * w * q1
then normalize.

This might be faster than computing sin+cos.
However, we can compute sin+cos of the same angle fast.
*/

// pooled locally, not thread-safe
// updated to rev 100
/**
 * This is an internal class.
 * @author Daniel Murphy
 */
public class Island {
	
	public ContactListener m_listener;

	public Body[] m_bodies;
	public Contact[] m_contacts;
	public Joint[] m_joints;

	public Position[] m_positions;
	public Velocity[] m_velocities;
	
	public int m_bodyCount;
	public int m_jointCount;
	public int m_contactCount;
	
	public int m_bodyCapacity;
	public int m_contactCapacity;
	public int m_jointCapacity;
	
	public int m_positionIterationCount;
	
	public Island(){
		
	}
	
	public void init(int bodyCapacity, int contactCapacity, int jointCapacity, ContactListener listener){
		m_bodyCapacity = bodyCapacity;
		m_contactCapacity = contactCapacity;
		m_jointCapacity = jointCapacity;
		m_bodyCount = 0;
		m_contactCount = 0;
		m_jointCount = 0;
		
		m_listener = listener;
		
		if(m_bodies == null || m_bodyCapacity >= m_bodies.length){
			m_bodies = new Body[m_bodyCapacity];
		}
		if(m_joints == null || m_jointCapacity > m_joints.length){
			m_joints = new Joint[m_jointCapacity];			
		}
		if(m_contacts == null || m_contactCapacity > m_contacts.length){
			m_contacts = new Contact[m_contactCapacity];
		}
		
		// could do some copy stuff, but just a full creation for now
		// TODO djm do a copy
		if(m_velocities == null || m_bodyCapacity > m_velocities.length){
			m_velocities = new Velocity[m_bodyCapacity];
			for(int i=0; i<m_velocities.length; i++){
				m_velocities[i] = new Velocity();
			}
		}
		
		// could do some copy stuff, but just a full creation for now
		// TODO djm do a copy
		if(m_positions == null || m_bodyCapacity > m_positions.length){
			m_positions = new Position[m_bodyCapacity];
			for(int i=0; i<m_positions.length; i++){
				m_positions[i] = new Position();
			}
		}
	}
	
	public void clear(){
		m_bodyCount = 0;
		m_contactCount = 0;
		m_jointCount = 0;
	}
	
	private final Vec2 temp = new Vec2();
	private final ContactSolver contactSolver = new ContactSolver();
	private final Vec2 translation = new Vec2();
	
	public void solve(TimeStep step, Vec2 gravity, boolean allowSleep){
		// Integrate velocities and apply damping.
		for (int i = 0; i < m_bodyCount; ++i){
			Body b = m_bodies[i];

			if (b.getType() != BodyType.DYNAMIC){
				continue;
			}
			

			// Integrate velocities.
			///b.m_linearVelocity += step.dt * (gravity + b.m_invMass * b.m_force);
//			temp.set(b.m_force).mulLocal(b.m_invMass).addLocal(gravity).mulLocal(step.dt);
//			b.m_linearVelocity.addLocal(temp);
//			b.m_angularVelocity += step.dt * b.m_invI * b.m_torque;
			
			b.m_linearVelocity.x += (b.m_force.x * b.m_invMass + gravity.x)*step.dt;
			b.m_linearVelocity.y += (b.m_force.y * b.m_invMass + gravity.y)*step.dt;
			b.m_angularVelocity += step.dt * b.m_invI * b.m_torque;
			

			// Apply damping.
			// ODE: dv/dt + c * v = 0
			// Solution: v(t) = v0 * exp(-c * t)
			// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
			// v2 = exp(-c * dt) * v1
			// Taylor expansion:
			// v2 = (1.0f - c * dt) * v1
//			b.m_linearVelocity.mulLocal(MathUtils.clamp(1.0f - step.dt * b.m_linearDamping, 0.0f, 1.0f));
//			b.m_angularVelocity *= MathUtils.clamp(1.0f - step.dt * b.m_angularDamping, 0.0f, 1.0f);
			
			
			float a = (1.0f - step.dt * b.m_linearDamping);
			float a1 = (0.0f > (a < 1.0f ? a : 1.0f) ? 0.0f : (a < 1.0f ? a : 1.0f));
			b.m_linearVelocity.x *= a1;
			b.m_linearVelocity.y *= a1;
			
			float a2 = (1.0f - step.dt * b.m_angularDamping);
			float b1 = (a2 < 1.0f ? a2 : 1.0f);
			b.m_angularVelocity *= 0.0f > b1 ? 0.0f : b1;
		}

		// Partition contacts so that contacts with static bodies are solved last.
		int i1 = -1;
		for (int i2 = 0; i2 < m_contactCount; ++i2){
			Fixture fixtureA = m_contacts[i2].getFixtureA();
			Fixture fixtureB = m_contacts[i2].getFixtureB();
			Body bodyA = fixtureA.getBody();
			Body bodyB = fixtureB.getBody();
			boolean nonStatic = bodyA.getType() != BodyType.STATIC && bodyB.getType() != BodyType.STATIC;
			if (nonStatic){
				++i1;
				//Swap(m_contacts[i1], m_contacts[i2]);
				Contact temp = m_contacts[i1];
				m_contacts[i1] = m_contacts[i2];
				m_contacts[i2] = temp;
			}
		}

		// Initialize velocity constraints.
		contactSolver.init(m_contacts, m_contactCount, step.dtRatio);
		contactSolver.warmStart();
		
		for (int i = 0; i < m_jointCount; ++i){
			m_joints[i].initVelocityConstraints(step);
		}

		// Solve velocity constraints.
		for (int i = 0; i < step.velocityIterations; ++i){
			for (int j = 0; j < m_jointCount; ++j){
				m_joints[j].solveVelocityConstraints(step);
			}
			contactSolver.solveVelocityConstraints();
		}

		// Post-solve (store impulses for warm starting).
		contactSolver.storeImpulses();

		// Integrate positions.
		for (int i = 0; i < m_bodyCount; ++i){
			Body b = m_bodies[i];

			if (b.getType() == BodyType.STATIC){
				continue;
			}

			// Check for large velocities.
			translation.set(b.m_linearVelocity).mulLocal(step.dt);
			if (Vec2.dot(translation, translation) > Settings.maxTranslationSquared){
				float ratio = Settings.maxTranslation / translation.length();
				b.m_linearVelocity.mulLocal(ratio);
			}

			float rotation = step.dt * b.m_angularVelocity;
			if (rotation * rotation > Settings.maxRotationSquared)
			{
				float ratio = Settings.maxRotation / Math.abs(rotation);
				b.m_angularVelocity *= ratio;
			}

			// Store positions for continuous collision.
			b.m_sweep.c0.set(b.m_sweep.c);
			b.m_sweep.a0 = b.m_sweep.a;


			// Integrate
			//b.m_sweep.c += step.dt * b.m_linearVelocity;
			temp.set(b.m_linearVelocity).mulLocal(step.dt);
			b.m_sweep.c.addLocal(temp);
			b.m_sweep.a += step.dt * b.m_angularVelocity;

			// Compute new transform
			b.synchronizeTransform();

			// Note: shapes are synchronized later.
		}

		// Iterate over constraints.
		for (int i = 0; i < step.positionIterations; ++i){
			boolean contactsOkay = contactSolver.solvePositionConstraints(Settings.contactBaumgarte);

			boolean jointsOkay = true;
			for (int j = 0; j < m_jointCount; ++j){
				boolean jointOkay = m_joints[j].solvePositionConstraints(Settings.contactBaumgarte);
				jointsOkay = jointsOkay && jointOkay;
			}

			if (contactsOkay && jointsOkay){
				// Exit early if the position errors are small.
				break;
			}
		}

		report(contactSolver.m_constraints);

		if (allowSleep){
			float minSleepTime = Float.MAX_VALUE;

			 float linTolSqr = Settings.linearSleepTolerance * Settings.linearSleepTolerance;
			 float angTolSqr = Settings.angularSleepTolerance * Settings.angularSleepTolerance;

			for (int i = 0; i < m_bodyCount; ++i){
				Body b = m_bodies[i];
				if (b.getType() == BodyType.STATIC){
					continue;
				}

				if ((b.m_flags & Body.e_autoSleepFlag) == 0){
					b.m_sleepTime = 0.0f;
					minSleepTime = 0.0f;
				}

				if ((b.m_flags & Body.e_autoSleepFlag) == 0 ||
					b.m_angularVelocity * b.m_angularVelocity > angTolSqr ||
					Vec2.dot(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr){
					b.m_sleepTime = 0.0f;
					minSleepTime = 0.0f;
				}
				else{
					b.m_sleepTime += step.dt;
					minSleepTime = MathUtils.min(minSleepTime, b.m_sleepTime);
				}
			}

			if (minSleepTime >= Settings.timeToSleep){
				for (int i = 0; i < m_bodyCount; ++i){
					Body b = m_bodies[i];
					b.setAwake(false);
				}
			}
		}
	}
	
	public void add(Body body){
		assert(m_bodyCount < m_bodyCapacity);
		body.m_islandIndex = m_bodyCount;
		m_bodies[m_bodyCount++] = body;
	}
	
	public void add(Contact contact){
		assert(m_contactCount < m_contactCapacity);
		m_contacts[m_contactCount++] = contact;
	}
	
	public void add(Joint joint){
		assert(m_jointCount < m_jointCapacity);
		m_joints[m_jointCount++] = joint;
	}
	
	private final ContactImpulse impulse = new ContactImpulse();
	
	public void report(ContactConstraint[] constraints){
		if (m_listener == null){
			return;
		}

		for (int i = 0; i < m_contactCount; ++i){
			Contact c = m_contacts[i];

			ContactConstraint cc = constraints[i];
			
			for (int j = 0; j < cc.pointCount; ++j){
				impulse.normalImpulses[j] = cc.points[j].normalImpulse;
				impulse.tangentImpulses[j] = cc.points[j].tangentImpulse;
			}

			m_listener.postSolve(c, impulse);
		}
	}
}

/**
 * This is an internal structure
 * @author Daniel
 */
class Position{
	final Vec2 x = new Vec2();
	float a;
}

/**
 * This is an internal structure
 * @author Daniel
 */
class Velocity{
	final Vec2 v = new Vec2();
	float a;
}

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

package org.jbox2d.dynamics;

import java.util.List;

import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.ManifoldPoint;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.contacts.ContactConstraint;
import org.jbox2d.dynamics.contacts.ContactConstraintPoint;
import org.jbox2d.dynamics.contacts.ContactResult;
import org.jbox2d.dynamics.contacts.ContactSolver;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.pooling.stacks.ContactSolverStack;

//Updated to rev. 46->103->142 of b2Island.cpp/.h

/**
 * Handles much of the heavy lifting of physics solving - for internal use.
 */
public class Island {
	public Body[] m_bodies;

	public Contact[] m_contacts;

	public Joint[] m_joints;

	public int m_bodyCount;

	public int m_jointCount;

	public int m_contactCount;

	public int m_bodyCapacity;

	public int m_contactCapacity;

	public int m_jointCapacity;

	public static int m_positionIterationCount = 0;

	public float m_positionError;

	public ContactListener m_listener;

	//begin .h methods
	public void clear() {
		m_bodyCount = 0;
		m_contactCount = 0;
		m_jointCount = 0;
	}

	void add(final Body body) {
		assert m_bodyCount < m_bodyCapacity;
		m_bodies[m_bodyCount++] = body;
	}

	void add(final Contact contact) {
		assert (m_contactCount < m_contactCapacity);
		m_contacts[m_contactCount++] = contact; //no clone, botches CCD if cloned!
	}

	void add(final Joint joint) {
		assert (m_jointCount < m_jointCapacity);
		m_joints[m_jointCount++] = joint;
	}
	//end .h methods

	//begin .cpp methods
	/**
	 * TODO djm: make this so it isn't created every time step
	 */
	public Island(){
		
	}
	
	public final void init(final int bodyCapacity,
	              final int contactCapacity,
	              final int jointCapacity,
	              final ContactListener listener) {

		m_bodyCapacity = bodyCapacity;
		m_contactCapacity = contactCapacity;
		m_jointCapacity	 = jointCapacity;
		m_bodyCount = 0;
		m_contactCount = 0;
		m_jointCount = 0;

		m_listener = listener;

		m_bodies = new Body[bodyCapacity];
		m_contacts = new Contact[contactCapacity];
		m_joints = new Joint[jointCapacity];

		m_positionIterationCount = 0;
	}

	// djm pooling
	private static final ContactSolverStack contactSolvers = new ContactSolverStack();
	
	public void solve(final TimeStep step, final Vec2 gravity, final boolean correctPositions, final boolean allowSleep) {
		// Integrate velocities and apply damping.
		for (int i = 0; i < m_bodyCount; ++i) {
			final Body b = m_bodies[i];

			if (b.isStatic()) {
				continue;
			}

			// Integrate velocities.
			b.m_linearVelocity.x += step.dt * (gravity.x + b.m_invMass * b.m_force.x);
			b.m_linearVelocity.y += step.dt * (gravity.y + b.m_invMass * b.m_force.y);
			b.m_angularVelocity += step.dt * b.m_invI * b.m_torque;

			// Reset forces.
			b.m_force.set(0.0f, 0.0f);
			b.m_torque = 0.0f;

			// Apply damping.
			// ODE: dv/dt + c * v = 0
			// Solution: v(t) = v0 * exp(-c * t)
			// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
			// v2 = exp(-c * dt) * v1
			// Taylor expansion:
			// v2 = (1.0f - c * dt) * v1
			b.m_linearVelocity.mulLocal(MathUtils.clamp(1.0f - step.dt * b.m_linearDamping, 0.0f, 1.0f));
			b.m_angularVelocity *= MathUtils.clamp(1.0f - step.dt * b.m_angularDamping, 0.0f, 1.0f);

			// Check for large velocities.
			if (Vec2.dot(b.m_linearVelocity, b.m_linearVelocity) > Settings.maxLinearVelocitySquared) {
				b.m_linearVelocity.normalize();
				b.m_linearVelocity.mulLocal(Settings.maxLinearVelocity);
			}

			if (b.m_angularVelocity * b.m_angularVelocity > Settings.maxAngularVelocitySquared) {
				if (b.m_angularVelocity < 0.0f) {
					b.m_angularVelocity = -Settings.maxAngularVelocity;
				} else {
					b.m_angularVelocity = Settings.maxAngularVelocity;
				}
			}
		}

		final ContactSolver contactSolver = contactSolvers.get();
		contactSolver.init(step, m_contacts, m_contactCount);

		// Initialize velocity constraints.
		contactSolver.initVelocityConstraints(step);

		for (int i = 0; i < m_jointCount; ++i) {
			m_joints[i].initVelocityConstraints(step);
		}

		// Solve velocity constraints.
		for (int i = 0; i < step.maxIterations; ++i) {
			contactSolver.solveVelocityConstraints();

			for (int j = 0; j < m_jointCount; ++j) {
				m_joints[j].solveVelocityConstraints(step);
			}
		}


		// Post-solve (store impulses for warm starting).
		contactSolver.finalizeVelocityConstraints();

		// Integrate positions.
		for (int i = 0; i < m_bodyCount; ++i) {
			final Body b = m_bodies[i];

			if (b.isStatic()) {
				continue;
			}

			// Store positions for continuous collision.
			b.m_sweep.c0.set(b.m_sweep.c);
			b.m_sweep.a0 = b.m_sweep.a;

			// Integrate
			b.m_sweep.c.x += step.dt * b.m_linearVelocity.x;
			b.m_sweep.c.y += step.dt * b.m_linearVelocity.y;
			b.m_sweep.a += step.dt * b.m_angularVelocity;

			// Compute new transform
			b.synchronizeTransform();

			// Note: shapes are synchronized later.
		}

		if (correctPositions) {
			// Initialize position constraints.
			// Contacts don't need initialization.

			for (int i = 0; i < m_jointCount; ++i) {
				m_joints[i].initPositionConstraints();
			}


			// Iterate over constraints.
			for (m_positionIterationCount = 0; m_positionIterationCount < step.maxIterations; ++m_positionIterationCount) {
				final boolean contactsOkay = contactSolver.solvePositionConstraints(Settings.contactBaumgarte);

				boolean jointsOkay = true;
				for (int i = 0; i < m_jointCount; ++i) {
					final boolean jointOkay = m_joints[i].solvePositionConstraints();
					jointsOkay = jointsOkay && jointOkay;
				}

				if (contactsOkay && jointsOkay) {
					break;
				}
			}

		}

		report(contactSolver.m_constraints);

		if (allowSleep) {
			float minSleepTime = Float.MAX_VALUE;

			final float linTolSqr = Settings.linearSleepTolerance * Settings.linearSleepTolerance;
			final float angTolSqr = Settings.angularSleepTolerance * Settings.angularSleepTolerance;

			for (int i = 0; i < m_bodyCount; ++i) {
				final Body b = m_bodies[i];
				if (b.m_invMass == 0.0f) {
					continue;
				}

				/*if ((b.m_flags & Body.e_allowSleepFlag) == 0) {
					b.m_sleepTime = 0.0f;
					minSleepTime = 0.0f;
					djm: we don't need this, as the next if statement takes care of it.  thanks Edge!
				}*/

				if ((b.m_flags & Body.e_allowSleepFlag) == 0 ||
						b.m_angularVelocity * b.m_angularVelocity > angTolSqr ||
						Vec2.dot(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr) {
					b.m_sleepTime = 0.0f;
					minSleepTime = 0.0f;
				} else {
					b.m_sleepTime += step.dt;
					minSleepTime = MathUtils.min(minSleepTime, b.m_sleepTime);
				}
			}

			if (minSleepTime >= Settings.timeToSleep) {
				for (int i = 0; i < m_bodyCount; ++i) {
					final Body b = m_bodies[i];
					b.m_flags |= Body.e_sleepFlag;
					// thanks Edge!
					b.m_linearVelocity.setZero(); // no new creation = new Vec2(0.0f, 0.0f);
					b.m_angularVelocity = 0.0f;
				}
			}
		}
		
		contactSolvers.recycle(contactSolver);
	}
	
	

	// djm pooling, from above
	public void solveTOI(final TimeStep subStep) {
		final ContactSolver contactSolver = contactSolvers.get();
		contactSolver.init(subStep, m_contacts, m_contactCount);

		// No warm starting needed for TOI contact events.

		// For joints, initialize with the last full step warm starting values
		if (Settings.maxTOIJointsPerIsland > 0) {
			subStep.warmStarting = true;
			//for (int i=0; i<m_jointCount; ++i) {
			for (int i = m_jointCount-1; i >= 0; --i) {
				m_joints[i].initVelocityConstraints(subStep);
			}

			// ...but don't update the warm starting value during solving
			subStep.warmStarting = false;
		}

		// Solve velocity constraints.
		for (int i = 0; i < subStep.maxIterations; ++i) {
			contactSolver.solveVelocityConstraints();
			//for (int j = 0; j < m_jointCount; ++j) {
			for (int j = m_jointCount-1; j >= 0; --j) {
				m_joints[j].solveVelocityConstraints(subStep);
			}
		}

		// Don't store the TOI contact forces for warm starting
		// because they can be quite large.

		// Integrate positions.
		for (int i = 0; i < m_bodyCount; ++i) {
			final Body b = m_bodies[i];

			if (b.isStatic()) {
				continue;
			}
			//System.out.println("(Island::SolveTOI 1) :"+b.m_sweep);
			// Store positions for continuous collision.
			b.m_sweep.c0.set(b.m_sweep.c);
			b.m_sweep.a0 = b.m_sweep.a;

			// Integrate
			b.m_sweep.c.x += subStep.dt * b.m_linearVelocity.x;
			b.m_sweep.c.y += subStep.dt * b.m_linearVelocity.y;
			b.m_sweep.a += subStep.dt * b.m_angularVelocity;

			//System.out.println("(Island::SolveTOI 2) :"+b.m_sweep);
			// Compute new transform
			b.synchronizeTransform();

			//	System.out.println("(Island::SolveTOI 3) :"+b.m_sweep);
			// Note: shapes are synchronized later.
		}

		// Solve position constraints.
		final float k_toiBaumgarte = 0.75f;
		for (int i = 0; i < subStep.maxIterations; ++i) {
			final boolean contactsOkay = contactSolver.solvePositionConstraints(k_toiBaumgarte);

			boolean jointsOkay = true;
			//for (int j = 0; j < m_jointCount; ++j) {
			for (int j = m_jointCount-1; j >= 0; --j) {
				final boolean jointOkay = m_joints[j].solvePositionConstraints();
				//System.out.println("iter "+i + ": "+j + " " + jointOkay);
				jointsOkay = jointsOkay && jointOkay;
			}

			if (contactsOkay && jointsOkay) {
				break;
			}
		}

		report(contactSolver.m_constraints);
		
		contactSolvers.recycle(contactSolver);
	}

	public void report(final List<ContactConstraint> constraints) {
		//TODO: optimize this, it's crummy
		final ContactConstraint[] cc = new ContactConstraint[constraints.size()];
		for (int i=0; i<cc.length; ++i) {
			cc[i] = constraints.get(i);
		}
		report(cc);
	}

	public void report(final ContactConstraint[] constraints) {
		if (m_listener == null) {
			return;
		}

		for (int i = 0; i < m_contactCount; ++i) {
			final Contact c = m_contacts[i];
			final ContactConstraint cc = constraints[i];
			final ContactResult cr = new ContactResult();
			cr.shape1 = c.getShape1();
			cr.shape2 = c.getShape2();
			final Body b1 = cr.shape1.getBody();
			final int manifoldCount = c.getManifoldCount();
			final List<Manifold> manifolds = c.getManifolds();
			for (int j = 0; j < manifoldCount; ++j) {
				final Manifold manifold = manifolds.get(j);
				cr.normal.set(manifold.normal);
				for (int k = 0; k < manifold.pointCount; ++k) {
					final ManifoldPoint point = manifold.points[k];
					final ContactConstraintPoint ccp = cc.points[k];
					XForm.mulToOut(b1.getMemberXForm(), point.localPoint1, cr.position);

					// TOI constraint results are not stored, so get
					// the result from the constraint.
					cr.normalImpulse = ccp.normalImpulse;
					cr.tangentImpulse = ccp.tangentImpulse;
					cr.id.set(point.id);

					m_listener.result(cr);
				}
			}
		}
	}
}

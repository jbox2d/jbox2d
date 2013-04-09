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

package org.jbox2d.dynamics.contacts;

import java.util.List;

import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.ManifoldPoint;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;


//Updated to rev 131->149 of b2ContactSolver.cpp/.h
// TODO djm: make this not be created all the time
public class ContactSolver {
	public TimeStep m_step;

	/*
	 * ContactSolver is often a bottleneck, so let's see if
	 * a plain array is faster than a List...
	 */
	//*
	public ContactConstraint[] m_constraints;
	/*/
	public List<ContactConstraint> m_constraints;
	//*/

	public int m_constraintCount;

	public ContactSolver(){
		
	}
	
	public final void init(final TimeStep step, final Contact[] contacts, final int contactCount) {
		m_step = step;

		m_constraintCount = 0;
		for (int i = 0; i < contactCount; i++) {// Contact c : contacts) {
			assert(contacts[i].isSolid());
			m_constraintCount += contacts[i].getManifoldCount();
		}

		m_constraints = new ContactConstraint[m_constraintCount];
		for (int i = 0; i < m_constraintCount; i++) {
			m_constraints[i] = new ContactConstraint();
		}

		int count = 0;
		for (int i = 0; i < contactCount; i++) {// Contact contact : contacts) {
			final Contact contact = contacts[i];

			final Body b1 = contact.m_shape1.getBody();
			final Body b2 = contact.m_shape2.getBody();
			final int manifoldCount = contact.getManifoldCount();
			final List<Manifold> manifolds = contact.getManifolds();
			final float friction = contact.m_friction;
			final float restitution = contact.m_restitution;

			final Vec2 v1 = b1.m_linearVelocity;//.clone(); //Not altered, no reason to clone
			final Vec2 v2 = b2.m_linearVelocity;//.clone();
			final float w1 = b1.m_angularVelocity;
			final float w2 = b2.m_angularVelocity;

			for (int j = 0; j < manifoldCount; ++j) {
				final Manifold manifold = manifolds.get(j);

				assert (manifold.pointCount > 0) : "Manifold " + j
				+ " has length 0";

				final Vec2 normal = manifold.normal;//.clone(); //not altered, no reason to clone

				assert (count < m_constraintCount);

				final ContactConstraint c = m_constraints[count];

				c.body1 = b1;
				c.body2 = b2;
				// DO NOT USE SET METHOD this object needs to be
				// referenced in the list
				c.manifold = manifold; //no copy here!
				c.normal.set(normal);// = normal.clone();
				c.pointCount = manifold.pointCount;

				c.friction = friction;
				c.restitution = restitution;

				for (int k = 0; k < c.pointCount; ++k) {
					final ManifoldPoint cp = manifold.points[k];
					final ContactConstraintPoint ccp = c.points[k];

					ccp.normalImpulse = cp.normalImpulse;
					ccp.tangentImpulse = cp.tangentImpulse;
					ccp.separation = cp.separation;
					ccp.positionImpulse = 0.0f;

					ccp.localAnchor1.set(cp.localPoint1);
					ccp.localAnchor2.set(cp.localPoint2);
					// INLINED
					//ccp.r1 = Mat22.mul(b1.getXForm().R, cp.localPoint1.sub(b1.getLocalCenter()));
					//ccp.r2 = Mat22.mul(b2.getXForm().R, cp.localPoint2.sub(b2.getLocalCenter()));
					final float v3x = cp.localPoint1.x - b1.m_sweep.localCenter.x;
					final float v3y = cp.localPoint1.y - b1.m_sweep.localCenter.y;
					ccp.r1.set(b1.m_xf.R.col1.x * v3x + b1.m_xf.R.col2.x * v3y, b1.m_xf.R.col1.y * v3x + b1.m_xf.R.col2.y * v3y);
					final float v4x = cp.localPoint2.x - b2.m_sweep.localCenter.x;
					final float v4y = cp.localPoint2.y - b2.m_sweep.localCenter.y;
					ccp.r2.set(b2.m_xf.R.col1.x * v4x + b2.m_xf.R.col2.x * v4y, b2.m_xf.R.col1.y * v4x + b2.m_xf.R.col2.y * v4y);

					float rn1 = Vec2.cross(ccp.r1, normal);
					float rn2 = Vec2.cross(ccp.r2, normal);
					rn1 *= rn1;
					rn2 *= rn2;

					final float kNormal = b1.m_invMass + b2.m_invMass + b1.m_invI * rn1 + b2.m_invI * rn2;

					assert (kNormal > Settings.EPSILON):"kNormal was "+kNormal;
					ccp.normalMass = 1.0f / kNormal;

					float kEqualized = b1.m_mass * b1.m_invMass + b2.m_mass * b2.m_invMass;
					kEqualized += b1.m_mass * b1.m_invI * rn1 + b2.m_mass * b2.m_invI * rn2;

					assert(kEqualized > Settings.EPSILON):"kEqualized was "+kEqualized;
					ccp.equalizedMass = 1.0f / kEqualized;

					//Vec2 tangent = Vec2.cross(normal, 1.0f);
					final float tangentx = normal.y;
					final float tangenty = -normal.x;

					final Vec2 a = ccp.r1;

					float rt1 = a.x * tangenty - a.y * tangentx;
					final Vec2 a1 = ccp.r2;//Vec2.cross(ccp.r1, tangent);
					float rt2 = a1.x * tangenty - a1.y * tangentx;//Vec2.cross(ccp.r2, tangent);
					rt1 *= rt1;
					rt2 *= rt2;

					final float kTangent = b1.m_invMass + b2.m_invMass + b1.m_invI * rt1 + b2.m_invI * rt2;


					assert (kTangent > Settings.EPSILON);
					ccp.tangentMass = 1.0f / kTangent;

					// Setup a velocity bias for restitution.
					ccp.velocityBias = 0.0f;
					if (ccp.separation > 0.0f) {
						ccp.velocityBias = -60.0f * ccp.separation; // TODO_ERIN b2TimeStep
					}
					final Vec2 a2 = ccp.r2;
					final Vec2 a3 = ccp.r1;
					// INLINED
					//Vec2 buffer = Vec2.cross(w2, ccp.r2).subLocal(Vec2.cross(w1, ccp.r1)).addLocal(v2).subLocal(v1);
					//float vRel = Vec2.dot(c.normal, buffer);
					final float bufferx = -w2 * a2.y - (-w1 * a3.y) + v2.x - v1.x;
					final float buffery = w2 * a2.x - w1 * a3.x + v2.y - v1.y;
					final float vRel = c.normal.x * bufferx + c.normal.y * buffery;
					if (vRel < -Settings.velocityThreshold) {
						ccp.velocityBias += -c.restitution * vRel;
					}

				}

				++count;
			}
		}

		assert (count == m_constraintCount);
	}

	public void initVelocityConstraints(final TimeStep step) {
		// Zero temp objects created - ewjordan

		// Warm start.
		for (int i = 0; i < m_constraintCount; ++i) {

			//*
			final ContactConstraint c = m_constraints[i];
			/*/
        	ContactConstraint c = m_constraints.get(i);
        	//*/

			final Body b1 = c.body1;
			final Body b2 = c.body2;
			final float invMass1 = b1.m_invMass;
			final float invI1 = b1.m_invI;
			final float invMass2 = b2.m_invMass;
			final float invI2 = b2.m_invI;
			final float normalx = c.normal.x;
			final float normaly = c.normal.y;
			final float tangentx = normaly;
			final float tangenty = -normalx;

			if (step.warmStarting) {

				for (int j = 0; j < c.pointCount; ++j) {
					final ContactConstraintPoint ccp = c.points[j];

					//Inlined all vector ops here
					ccp.normalImpulse *= step.dtRatio;
					ccp.tangentImpulse *= step.dtRatio;

					final float px = (ccp.normalImpulse * normalx + ccp.tangentImpulse * tangentx);
					final float py = (ccp.normalImpulse * normaly + ccp.tangentImpulse * tangenty);

					b1.m_angularVelocity -= invI1 * (ccp.r1.x * py - ccp.r1.y * px);
					b1.m_linearVelocity.x -= px * invMass1;
					b1.m_linearVelocity.y -= py * invMass1;
					b2.m_angularVelocity += invI2 * (ccp.r2.x * py - ccp.r2.y * px);
					b2.m_linearVelocity.x += px * invMass2;
					b2.m_linearVelocity.y += py * invMass2;
				}

			} else {
				for (int j = 0; j < c.pointCount; ++j) {
					final ContactConstraintPoint ccp = c.points[j];
					ccp.normalImpulse = 0.0f;
					ccp.tangentImpulse = 0.0f;
				}
			}
		}
	}

	public void solveVelocityConstraints() {
		// ewj: now clean of temp objects
		for (int i=0; i<this.m_constraintCount; ++i) {

			//*
			final ContactConstraint c = m_constraints[i];
			/*/
        	ContactConstraint c = m_constraints.get(i);
        	//*/
			final Body b1 = c.body1;
			final Body b2 = c.body2;
			float w1 = b1.m_angularVelocity;
			float w2 = b2.m_angularVelocity;
			//Vec2 v1 = b1.m_linearVelocity.clone();
			//Vec2 v2 = b2.m_linearVelocity.clone();
			float v1x = b1.m_linearVelocity.x;
			float v1y = b1.m_linearVelocity.y;
			float v2x = b2.m_linearVelocity.x;
			float v2y = b2.m_linearVelocity.y;
			final float invMass1 = b1.m_invMass;
			final float invI1 = b1.m_invI;
			final float invMass2 = b2.m_invMass;
			final float invI2 = b2.m_invI;
			//Vec2 normal = c.normal;//.clone();
			final float normalx = c.normal.x;
			final float normaly = c.normal.y;
			//Vec2 tangent = Vec2.cross(normal, 1.0f);
			final float tangentx = normaly;
			final float tangenty = -normalx;
			final float friction = c.friction;

			//final boolean DEFERRED_UPDATE = false;
			//if (DEFERRED_UPDATE) {
			//            		Vec2 b1_linearVelocity = b1.m_linearVelocity.clone();
			//            		float b1_angularVelocity = b1.m_angularVelocity;
			//            		Vec2 b2_linearVelocity = b2.m_linearVelocity.clone();
			//            		float b2_angularVelocity = b2.m_angularVelocity;
			//}

			// Solver normal constraints
			for (int j=0; j<c.pointCount; ++j) {

				final ContactConstraintPoint ccp = c.points[j];

				// Relative velocity at contact
				//Vec2 dv = v2.add(Vec2.cross(w2,ccp.r2));
				//dv.subLocal(v1);
				//Vec2 a = ccp.r1;
				//dv.subLocal(new Vec2(-w1 * a.y, w1 * a.x));
				final float dvx = v2x - w2 * ccp.r2.y - v1x + w1*ccp.r1.y;
				final float dvy = v2y + w2 * ccp.r2.x - v1y - w1*ccp.r1.x;

				// Compute normal impulse
				final float vn = dvx*normalx + dvy*normaly;//Vec2.dot(dv, normal);
				float lambda = - ccp.normalMass * (vn - ccp.velocityBias);

				// b2Clamp the accumulated force
				final float newImpulse = MathUtils.max(ccp.normalImpulse + lambda, 0.0f);
				lambda = newImpulse - ccp.normalImpulse;

				// Apply contact impulse
				//Vec2 P = new Vec2(lambda * normal.x, lambda * normal.y);
				final float Px = lambda * normalx;
				final float Py = lambda * normaly;

				v1x -= invMass1*Px;
				v1y -= invMass1*Py;
				w1 -= invI1 * (ccp.r1.x * Py - ccp.r1.y * Px);
				//Vec2.cross(ccp.r1,P);

				v2x += invMass2*Px;
				v2y += invMass2*Py;
				w2 += invI2 * (ccp.r2.x * Py - ccp.r2.y * Px);
				//Vec2.cross(ccp.r2,P);

				ccp.normalImpulse = newImpulse;

			}

			//            //#ifdef DEFERRED_UPDATE
			//    		b1.m_linearVelocity = b1_linearVelocity;
			//    		b1.m_angularVelocity = b1_angularVelocity;
			//    		b2.m_linearVelocity = b2_linearVelocity;
			//    		b2.m_angularVelocity = b2_angularVelocity;
			//    		// #endif

			// Solver tangent constraints
			for (int j=0; j<c.pointCount; ++j) {
				//ContactConstraintPoint ccp : c.points) {
				final ContactConstraintPoint ccp = c.points[j];

				// Relative velocity at contact
				//Vec2 dv = v2.add(Vec2.cross(w2, ccp.r2));
				//dv.subLocal(v1);
				//dv.subLocal(Vec2.cross(w1,ccp.r1));
				final float dvx = v2x - w2 * ccp.r2.y - v1x + w1*ccp.r1.y;
				final float dvy = v2y + w2 * ccp.r2.x - v1y - w1*ccp.r1.x;

				// Compute tangent force
				final float vt = dvx * tangentx + dvy * tangenty;
				float lambda = ccp.tangentMass * (-vt);

				// b2Clamp the accumulated force
				final float maxFriction = friction * ccp.normalImpulse;
				final float newImpulse = MathUtils.max(-maxFriction, MathUtils.min(ccp.tangentImpulse + lambda, maxFriction));
				lambda = newImpulse - ccp.tangentImpulse;

				// Apply contact impulse
				//Vec2 P = lambda * tangent;
				final float px = lambda * tangentx;
				final float py = lambda * tangenty;

				// b1.m_linearVelocity.subLocal(P.mul(invMass1));
				v1x -= px * invMass1;
				v1y -= py * invMass1;
				// b1.m_angularVelocity -= invI1 * Vec2.cross(r1, P);
				w1 -= invI1 * (ccp.r1.x * py - ccp.r1.y * px);

				// b2.m_linearVelocity.addLocal(P.mul(invMass2));
				v2x += px * invMass2;
				v2y += py * invMass2;
				// b2.m_angularVelocity += invI2 * Vec2.cross(r2, P);
				w2 += invI2 * (ccp.r2.x * py - ccp.r2.y * px);

				ccp.tangentImpulse = newImpulse;
			}
			b1.m_linearVelocity.x = v1x;
			b1.m_linearVelocity.y = v1y;
			b1.m_angularVelocity = w1;
			b2.m_linearVelocity.x = v2x;
			b2.m_linearVelocity.y = v2y;
			b2.m_angularVelocity = w2;
		}
	}

	public void finalizeVelocityConstraints() {
		for (int i = 0; i < m_constraintCount; ++i) {
			//*
			final ContactConstraint c = m_constraints[i];
			/*/
        	ContactConstraint c = m_constraints.get(i);
        	//*/
			final Manifold m = c.manifold;

			for (int j = 0; j < c.pointCount; ++j)
			{
				m.points[j].normalImpulse = c.points[j].normalImpulse;
				m.points[j].tangentImpulse = c.points[j].tangentImpulse;
			}
		}
	}

	public boolean solvePositionConstraints(final float baumgarte) {

		float minSeparation = 0.0f;
		for (int i=0; i<this.m_constraintCount; ++i) {
			//ContactConstraint c : m_constraints) {
			//*
			final ContactConstraint c = m_constraints[i];
			/*/
        	ContactConstraint c = m_constraints.get(i);
        	//*/

			final Body b1 = c.body1;
			final Body b2 = c.body2;
			final float invMass1 = b1.m_mass * b1.m_invMass;
			final float invI1 = b1.m_mass * b1.m_invI;
			final float invMass2 = b2.m_mass * b2.m_invMass;
			final float invI2 = b2.m_mass * b2.m_invI;

			final Vec2 normal = c.normal;

			// Solver normal constraints
			for (int j = 0; j < c.pointCount; ++j) {
				final ContactConstraintPoint ccp = c.points[j];

				//Vec2 r1 = Mat22.mul(b1.getXForm().R, ccp.localAnchor1.sub(b1.getLocalCenter()));
				//Vec2 r2 = Mat22.mul(b2.getXForm().R, ccp.localAnchor2.sub(b2.getLocalCenter()));
				//Vec2 r1 = Mat22.mul(b1.m_xf.R, ccp.localAnchor1.sub(b1.m_sweep.localCenter));
				//Vec2 r2 = Mat22.mul(b2.m_xf.R, ccp.localAnchor2.sub(b2.m_sweep.localCenter));
				//Vec2 v = ccp.localAnchor1.sub(b1.m_sweep.localCenter);
				float vx = ccp.localAnchor1.x - b1.m_sweep.localCenter.x;
				float vy = ccp.localAnchor1.y - b1.m_sweep.localCenter.y;
				final float r1x = b1.m_xf.R.col1.x * vx + b1.m_xf.R.col2.x * vy;
				final float r1y = b1.m_xf.R.col1.y * vx + b1.m_xf.R.col2.y * vy;
				vx = ccp.localAnchor2.x - b2.m_sweep.localCenter.x;
				vy = ccp.localAnchor2.y - b2.m_sweep.localCenter.y;
				final float r2x = b2.m_xf.R.col1.x * vx + b2.m_xf.R.col2.x * vy;
				final float r2y = b2.m_xf.R.col1.y * vx + b2.m_xf.R.col2.y * vy;

				//Vec2 p1 = b1.m_sweep.c + r1;
				//Vec2 p2 = b2.m_sweep.c + r2;
				//Vec2 dp = p2 - p1;
				final float dpx = b2.m_sweep.c.x + r2x - b1.m_sweep.c.x - r1x;
				final float dpy = b2.m_sweep.c.y + r2y - b1.m_sweep.c.y - r1y;


				// Approximate the current separation.
				final float separation = dpx*normal.x + dpy*normal.y + ccp.separation;//b2Dot(dp, normal) + ccp->separation;

				// Track max constraint error.
				minSeparation = MathUtils.min(minSeparation, separation);

				// Prevent large corrections and allow slop.
				final float C = baumgarte * MathUtils.clamp(separation + Settings.linearSlop, -Settings.maxLinearCorrection, 0.0f);

				// Compute normal impulse
				float dImpulse = -ccp.equalizedMass * C;

				// b2Clamp the accumulated impulse
				final float impulse0 = ccp.positionImpulse;
				ccp.positionImpulse = MathUtils.max(impulse0 + dImpulse, 0.0f);
				dImpulse = ccp.positionImpulse - impulse0;

				final float impulsex = dImpulse * normal.x;
				final float impulsey = dImpulse * normal.y;

				b1.m_sweep.c.x -= invMass1 * impulsex;
				b1.m_sweep.c.y -= invMass1 * impulsey;
				b1.m_sweep.a -= invI1 * (r1x*impulsey - r1y*impulsex);//b2Cross(r1, impulse);
				b1.synchronizeTransform();

				b2.m_sweep.c.x += invMass2 * impulsex;
				b2.m_sweep.c.y += invMass2 * impulsey;
				b2.m_sweep.a += invI2 * (r2x*impulsey - r2y*impulsex);//b2Cross(r2, impulse);
				b2.synchronizeTransform();
			}
		}

		// We can't expect minSpeparation >= -b2_linearSlop because we don't
		// push the separation above -b2_linearSlop.
		return minSeparation >= -1.5f * Settings.linearSlop;
	}

}

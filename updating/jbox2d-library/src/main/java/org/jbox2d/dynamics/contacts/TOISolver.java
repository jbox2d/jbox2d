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
 * Created at 2:25:52 PM Jul 6, 2010
 */
package org.jbox2d.dynamics.contacts;

import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.structs.collision.ManifoldPoint;

// updated to rev 100
// pooling: local, non-thread
/**
 * This is a pure position solver for a single movable body in contact with
 * multiple non-moving bodies.
 * @author daniel
 */
public class TOISolver {
	// TODO djm: find out the best number to start with
	private TOIConstraint[] m_constraints = new TOIConstraint[4];
	private int m_count;
	private Body m_toiBody;
	
	public TOISolver(){
		m_count = 0;
		m_toiBody = null;
		for(int i=0; i<m_constraints.length; i++){
			m_constraints[i] = new TOIConstraint();
		}
	}
	
	public void clear(){
		// does nothing
	}
	
	public void initialize(Contact[] contacts, int count, Body toiBody){
		//clear();
		
		m_count = count;
		m_toiBody = toiBody;
		
		// TODO djm: can I pool this? for now just having an expandable array
		if(m_count >= m_constraints.length){
			TOIConstraint[] old = m_constraints;
			m_constraints = new TOIConstraint[old.length*2];
			System.arraycopy(old, 0, m_constraints, 0, old.length);
			for(int i=old.length; i<m_constraints.length; i++){
				m_constraints[i] = new TOIConstraint();
			}
		}
//		m_constraints = new TOIConstraint[count];

		for(int i=0; i<m_count; i++){
			Contact contact = contacts[i];
			
			Fixture fixtureA = contact.getFixtureA();
			Fixture fixtureB = contact.getFixtureB();
			Shape shapeA = fixtureA.getShape();
			Shape shapeB = fixtureB.getShape();
			float radiusA = shapeA.m_radius;
			float radiusB = shapeB.m_radius;
			Body bodyA = fixtureA.getBody();
			Body bodyB = fixtureB.getBody();
			Manifold manifold = contact.getManifold();
			
			assert(manifold.pointCount > 0);
			
//			m_constraints[i] = new TOIConstraint();
			TOIConstraint constraint = m_constraints[i];
			constraint.bodyA = bodyA;
			constraint.bodyB = bodyB;
			constraint.localNormal.set(manifold.localNormal);
			constraint.localPoint.set(manifold.localPoint);
			constraint.type = manifold.type;
			constraint.pointCount = manifold.pointCount;
			constraint.radius = radiusA + radiusB;

			for (int j = 0; j < constraint.pointCount; ++j){
				ManifoldPoint cp = manifold.points[j];
				constraint.localPoints[j] = cp.localPoint;
			}
		}
	}
	
	// djm pooling
	private final TOISolverManifold psm = new TOISolverManifold();
	private final Vec2 rA = new Vec2();
	private final Vec2 rB = new Vec2();
	private final Vec2 P = new Vec2();
	private final Vec2 temp = new Vec2();
	/**
	 * Perform one solver iteration. Returns true if converged.
	 * @param baumgarte
	 * @return
	 */
	public boolean solve(float baumgarte){
		// Push out the toi body to provide clearance for further simulation.
		
		float minSeparation = 0f;
		
		for (int i = 0; i < m_count; ++i){
			TOIConstraint c = m_constraints[i];
			Body bodyA = c.bodyA;
			Body bodyB = c.bodyB;

			float massA = bodyA.m_mass;
			float massB = bodyB.m_mass;

			// Only the TOI body should move.
			if (bodyA == m_toiBody){
				massB = 0.0f;
			}
			else{
				massA = 0.0f;
			}

			float invMassA = massA * bodyA.m_invMass;
			float invIA = massA * bodyA.m_invI;
			float invMassB = massB * bodyB.m_invMass;
			float invIB = massB * bodyB.m_invI;

			// Solve normal constraints
			for (int j = 0; j < c.pointCount; ++j){
				psm.initialize(c, j);
				Vec2 normal = psm.normal;

				Vec2 point = psm.point;
				float separation = psm.separation;

				rA.set(point).subLocal(bodyA.m_sweep.c);
				rB.set(point).subLocal(bodyB.m_sweep.c);

				// Track max constraint error.
				minSeparation = MathUtils.min(minSeparation, separation);

				// Prevent large corrections and allow slop.
				float C = MathUtils.clamp(baumgarte * (separation + Settings.linearSlop), -Settings.maxLinearCorrection, 0.0f);

				// Compute the effective mass.
				float rnA = Vec2.cross(rA, normal);
				float rnB = Vec2.cross(rB, normal);
				float K = invMassA + invMassB + invIA * rnA * rnA + invIB * rnB * rnB;

				// Compute normal impulse
				float impulse = K > 0.0f ? - C / K : 0.0f;

				P.set(normal).mulLocal(impulse);

				temp.set(P).mulLocal(invMassA);
				bodyA.m_sweep.c.subLocal(temp);
				bodyA.m_sweep.a -= invIA * Vec2.cross(rA, P);
				bodyA.synchronizeTransform();

				temp.set(P).mulLocal(invMassB);
				bodyB.m_sweep.c.addLocal(temp);
				bodyB.m_sweep.a += invIB * Vec2.cross(rB, P);
				bodyB.synchronizeTransform();
			}
		}

		// We can't expect minSpeparation >= -_linearSlop because we don't
		// push the separation above -_linearSlop.
		return minSeparation >= -1.5f * Settings.linearSlop;
	}
}

class TOISolverManifold{
	public final Vec2 normal = new Vec2();
	public final Vec2 point = new Vec2();
	public float separation;
	
	// djm pooling
	private final Vec2 pointA = new Vec2();
	private final Vec2 pointB = new Vec2();
	private final Vec2 temp = new Vec2();
	private final Vec2 planePoint = new Vec2();
	private final Vec2 clipPoint = new Vec2();
	
	public void initialize(TOIConstraint cc, int index){
		assert(cc.pointCount > 0);

		switch (cc.type){
			case CIRCLES : {
				cc.bodyA.getWorldPointToOut(cc.localPoint, pointA);
				cc.bodyB.getWorldPointToOut(cc.localPoints[0], pointB);
				if (MathUtils.distanceSquared(pointA, pointB) > Settings.EPSILON * Settings.EPSILON) {
					normal.set(pointB).subLocal(pointA);
					normal.normalize();
				}
				else {
					normal.set(1.0f, 0.0f);
				}
				
				point.set(pointA).addLocal(pointB).mulLocal(.5f);
				temp.set(pointB).subLocal(pointA);
				separation = Vec2.dot(temp, normal) - cc.radius;
				break;
			}
			case FACE_A : {
				cc.bodyA.getWorldVectorToOut(cc.localNormal, normal);
				cc.bodyA.getWorldPointToOut(cc.localPoint, planePoint);
				
				cc.bodyB.getWorldPointToOut(cc.localPoints[index], clipPoint);
				temp.set(clipPoint).subLocal(planePoint);
				separation = Vec2.dot(temp, normal) - cc.radius;
				point.set(clipPoint);
				break;
			}
			
			case FACE_B : {
				cc.bodyB.getWorldVectorToOut(cc.localNormal, normal);
				cc.bodyB.getWorldPointToOut(cc.localPoint, planePoint);
				
				cc.bodyA.getWorldPointToOut(cc.localPoints[index], clipPoint);
				temp.set(clipPoint).subLocal(planePoint);
				separation = Vec2.dot(temp, normal) - cc.radius;
				point.set(clipPoint);
				
				// Ensure normal points from A to B
				normal.negateLocal();
			}
				break;
		}
	}
}

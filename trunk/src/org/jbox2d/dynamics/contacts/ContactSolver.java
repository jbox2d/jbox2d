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

import java.util.ArrayList;
import java.util.List;

import org.jbox2d.collision.ManifoldPoint;
import org.jbox2d.collision.Manifold;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.dynamics.World;


//Updated to rev 131->149 of b2ContactSolver.cpp/.h

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

    public ContactSolver(TimeStep step, Contact[] contacts, int contactCount) {
    	m_step = step;
    	
        m_constraintCount = 0;
        for (int i = 0; i < contactCount; i++) {// Contact c : contacts) {
        	assert(contacts[i].isSolid());
            m_constraintCount += contacts[i].getManifoldCount();
        }

        //*
        m_constraints = new ContactConstraint[m_constraintCount];
        /*/
        m_constraints = new ArrayList<ContactConstraint>(m_constraintCount);
        //*/
        for (int i = 0; i < m_constraintCount; i++) {
        	//*
        	m_constraints[i] = new ContactConstraint();
            /*/
        	m_constraints.add(new ContactConstraint());
        	//*/
        }

        int count = 0;
        for (int i = 0; i < contactCount; i++) {// Contact contact : contacts) {
            Contact contact = contacts[i];
            
            Body b1 = contact.m_shape1.getBody();
            Body b2 = contact.m_shape2.getBody();
            int manifoldCount = contact.getManifoldCount();
            List<Manifold> manifolds = contact.getManifolds();
            float friction = contact.m_friction;
            float restitution = contact.m_restitution;

            Vec2 v1 = b1.m_linearVelocity.clone();
            Vec2 v2 = b2.m_linearVelocity.clone();
            float w1 = b1.m_angularVelocity;
            float w2 = b2.m_angularVelocity;

            for (int j = 0; j < manifoldCount; ++j) {// Manifold manifold :
                // manifolds) {
                Manifold manifold = manifolds.get(j);

                assert (manifold.pointCount > 0) : "Manifold " + j
                        + " has length 0";

                Vec2 normal = manifold.normal.clone();

                assert (count < m_constraintCount);
                
                //*
                ContactConstraint c = m_constraints[count];
                /*/
                ContactConstraint c = m_constraints.get(count);
                //*/
                
                c.body1 = b1;
                c.body2 = b2;
                c.manifold = manifold; //no copy here!
                c.normal = normal.clone();
                c.pointCount = manifold.pointCount;
                
                c.friction = friction;
                c.restitution = restitution;

                for (int k = 0; k < c.pointCount; ++k) {
                    ManifoldPoint cp = manifold.points[k];
                    ContactConstraintPoint ccp = c.points[k];

                    ccp.normalImpulse = cp.normalImpulse;
                    ccp.tangentImpulse = cp.tangentImpulse;
                    ccp.separation = cp.separation;
                    ccp.positionImpulse = 0.0f;

                    ccp.localAnchor1.set(cp.localPoint1);
    				ccp.localAnchor2.set(cp.localPoint2);
    				ccp.r1 = Mat22.mul(b1.getXForm().R, cp.localPoint1.sub(b1.getLocalCenter()));
    				ccp.r2 = Mat22.mul(b2.getXForm().R, cp.localPoint2.sub(b2.getLocalCenter()));

    				float rn1 = Vec2.cross(ccp.r1, normal);
    				float rn2 = Vec2.cross(ccp.r2, normal);
    				rn1 *= rn1;
    				rn2 *= rn2;

    				float kNormal = b1.m_invMass + b2.m_invMass + b1.m_invI * rn1 + b2.m_invI * rn2;


                    assert (kNormal > Settings.EPSILON);
                    ccp.normalMass = 1.0f / kNormal;

                    float kEqualized = b1.m_mass * b1.m_invMass + b2.m_mass * b2.m_invMass;
                    kEqualized += b1.m_mass * b1.m_invI * rn1 + b2.m_mass * b2.m_invI * rn2;

    				assert(kEqualized > Settings.EPSILON);
    				ccp.equalizedMass = 1.0f / kEqualized;

                    Vec2 tangent = Vec2.cross(normal, 1.0f);

                    float rt1 = Vec2.cross(ccp.r1, tangent);
    				float rt2 = Vec2.cross(ccp.r2, tangent);
    				rt1 *= rt1;
    				rt2 *= rt2;

    				float kTangent = b1.m_invMass + b2.m_invMass + b1.m_invI * rt1 + b2.m_invI * rt2;

                    
                    assert (kTangent > Settings.EPSILON);
                    ccp.tangentMass = 1.0f / kTangent;

                    // Setup a velocity bias for restitution.
                    ccp.velocityBias = 0.0f;
                    if (ccp.separation > 0.0f) {
                        ccp.velocityBias = -60.0f * ccp.separation; // TODO_ERIN b2TimeStep
                    }
                    Vec2 buffer = Vec2.cross(w2, ccp.r2).subLocal(Vec2.cross(w1, ccp.r1)).addLocal(v2).subLocal(v1);
                    float vRel = Vec2.dot(c.normal, buffer);
                    if (vRel < -Settings.velocityThreshold) {
                    	ccp.velocityBias += -c.restitution * vRel;
                    }
                    
                }

                ++count;
            }
        }

        assert (count == m_constraintCount);
    }

    public void initVelocityConstraints(TimeStep step) {
    	// Zero temp objects created - ewjordan
    	
        // Warm start.
        for (int i = 0; i < m_constraintCount; ++i) {
            
        	//*
        	ContactConstraint c = m_constraints[i];
        	/*/
        	ContactConstraint c = m_constraints.get(i);
        	//*/
        	
            Body b1 = c.body1;
            Body b2 = c.body2;
            float invMass1 = b1.m_invMass;
            float invI1 = b1.m_invI;
            float invMass2 = b2.m_invMass;
            float invI2 = b2.m_invI;
            float normalx = c.normal.x;
            float normaly = c.normal.y;
            float tangentx = normaly;
            float tangenty = -normalx;

            if (step.warmStarting) {
            	
                for (int j = 0; j < c.pointCount; ++j) {
                    ContactConstraintPoint ccp = c.points[j];
                    
                    //Inlined all vector ops here
                    ccp.normalImpulse *= step.dtRatio;
    				ccp.tangentImpulse *= step.dtRatio;
                    
    				float px = (ccp.normalImpulse * normalx + ccp.tangentImpulse * tangentx);
                    float py = (ccp.normalImpulse * normaly + ccp.tangentImpulse * tangenty);
                    
                    b1.m_angularVelocity -= invI1 * (ccp.r1.x * py - ccp.r1.y * px);
                    b1.m_linearVelocity.x -= px * invMass1;
                    b1.m_linearVelocity.y -= py * invMass1;
                    b2.m_angularVelocity += invI2 * (ccp.r2.x * py - ccp.r2.y * px);
                    b2.m_linearVelocity.x += px * invMass2;
                    b2.m_linearVelocity.y += py * invMass2;
                }

            } else {
                for (int j = 0; j < c.pointCount; ++j) {
                    ContactConstraintPoint ccp = c.points[j];
                    ccp.normalImpulse = 0.0f;
                    ccp.tangentImpulse = 0.0f;
                }
            }
        }
    }

    public void solveVelocityConstraints() {
    	// (4*constraints + 6*points) temp Vec2s - BOTTLENECK!
    	
    	for (int i=0; i<this.m_constraintCount; ++i) {
    		
    		//*
        	ContactConstraint c = m_constraints[i];
        	/*/
        	ContactConstraint c = m_constraints.get(i);
        	//*/
            Body b1 = c.body1;
            Body b2 = c.body2;
            float w1 = b1.m_angularVelocity;
            float w2 = b2.m_angularVelocity;
            Vec2 v1 = b1.m_linearVelocity.clone();
            Vec2 v2 = b2.m_linearVelocity.clone();
            float invMass1 = b1.m_invMass;
            float invI1 = b1.m_invI;
            float invMass2 = b2.m_invMass;
            float invI2 = b2.m_invI;
            Vec2 normal = c.normal;//.clone();
            Vec2 tangent = Vec2.cross(normal, 1.0f);
            float friction = c.friction;
            
            //final boolean DEFERRED_UPDATE = false;
            //if (DEFERRED_UPDATE) {
//            		Vec2 b1_linearVelocity = b1.m_linearVelocity.clone();
//            		float b1_angularVelocity = b1.m_angularVelocity;
//            		Vec2 b2_linearVelocity = b2.m_linearVelocity.clone();
//            		float b2_angularVelocity = b2.m_angularVelocity;
            //}
            
            // Solver normal constraints
            for (int j=0; j<c.pointCount; ++j) {
            	
            	ContactConstraintPoint ccp = c.points[j];
            	
                // Relative velocity at contact
                //Vec2 dv = v2.add(Vec2.cross(w2,ccp.r2));
                //dv.subLocal(v1);
				//Vec2 a = ccp.r1;
                //dv.subLocal(new Vec2(-w1 * a.y, w1 * a.x));
                float dvx = v2.x - w2 * ccp.r2.y - v1.x + w1*ccp.r1.y;
                float dvy = v2.y + w2 * ccp.r2.x - v1.y - w1*ccp.r1.x;
            	
    			// Compute normal impulse
    			float vn = dvx*normal.x + dvy*normal.y;//Vec2.dot(dv, normal);
    			float lambda = - ccp.normalMass * (vn - ccp.velocityBias);

    			// b2Clamp the accumulated force
    			float newImpulse = MathUtils.max(ccp.normalImpulse + lambda, 0.0f);
    			lambda = newImpulse - ccp.normalImpulse;

    			// Apply contact impulse
    			//Vec2 P = new Vec2(lambda * normal.x, lambda * normal.y);
    			float Px = lambda * normal.x;
    			float Py = lambda * normal.y;
    			
    			v1.x -= invMass1*Px;
    			v1.y -= invMass1*Py;
    			w1 -= invI1 * (ccp.r1.x * Py - ccp.r1.y * Px); 
    							//Vec2.cross(ccp.r1,P);
    			
    			v2.x += invMass2*Px;
    			v2.y += invMass2*Py;
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
    			 ContactConstraintPoint ccp = c.points[j];

                // Relative velocity at contact
                //Vec2 dv = v2.add(Vec2.cross(w2, ccp.r2));
                //dv.subLocal(v1);
                //dv.subLocal(Vec2.cross(w1,ccp.r1));
                float dvx = v2.x - w2 * ccp.r2.y - v1.x + w1*ccp.r1.y;
                float dvy = v2.y + w2 * ccp.r2.x - v1.y - w1*ccp.r1.x;

                // Compute tangent force
    			float vt = dvx * tangent.x + dvy * tangent.y;
    			float lambda = ccp.tangentMass * (-vt);

    			// b2Clamp the accumulated force
    			float maxFriction = friction * ccp.normalImpulse;
    			float newImpulse = MathUtils.max(-maxFriction, MathUtils.min(ccp.tangentImpulse + lambda, maxFriction));
    			lambda = newImpulse - ccp.tangentImpulse;

    			// Apply contact impulse
    			//Vec2 P = lambda * tangent;
    			float px = lambda * tangent.x;
    			float py = lambda * tangent.y;

                // b1.m_linearVelocity.subLocal(P.mul(invMass1));
                v1.x -= px * invMass1;
                v1.y -= py * invMass1;
                // b1.m_angularVelocity -= invI1 * Vec2.cross(r1, P);
                w1 -= invI1 * (ccp.r1.x * py - ccp.r1.y * px);

                // b2.m_linearVelocity.addLocal(P.mul(invMass2));
                v2.x += px * invMass2;
                v2.y += py * invMass2;
                // b2.m_angularVelocity += invI2 * Vec2.cross(r2, P);
                w2 += invI2 * (ccp.r2.x * py - ccp.r2.y * px);

                ccp.tangentImpulse = newImpulse;
            }
    		 b1.m_linearVelocity.set(v1);
    		 b1.m_angularVelocity = w1;
    		 b2.m_linearVelocity.set(v2);
    		 b2.m_angularVelocity = w2;
        }
    }

    public void finalizeVelocityConstraints() {
    	for (int i = 0; i < m_constraintCount; ++i) {
    		//*
        	ContactConstraint c = m_constraints[i];
        	/*/
        	ContactConstraint c = m_constraints.get(i);
        	//*/
    		Manifold m = c.manifold;

    		for (int j = 0; j < c.pointCount; ++j)
    		{
    			m.points[j].normalImpulse = c.points[j].normalImpulse;
    			m.points[j].tangentImpulse = c.points[j].tangentImpulse;
    		}
    	}
    }
    
    public boolean solvePositionConstraints(float baumgarte) {
        float minSeparation = 0.0f;
        for (int i=0; i<this.m_constraintCount; ++i) {
			//ContactConstraint c : m_constraints) {
        	//*
        	ContactConstraint c = m_constraints[i];
        	/*/
        	ContactConstraint c = m_constraints.get(i);
        	//*/
        
			Body b1 = c.body1;
            Body b2 = c.body2;
            float invMass1 = b1.m_mass * b1.m_invMass;
    		float invI1 = b1.m_mass * b1.m_invI;
    		float invMass2 = b2.m_mass * b2.m_invMass;
    		float invI2 = b2.m_mass * b2.m_invI;
    		
    		Vec2 normal = c.normal;

    		// Solver normal constraints
    		for (int j = 0; j < c.pointCount; ++j) {
    			ContactConstraintPoint ccp = c.points[j];

    			Vec2 r1 = Mat22.mul(b1.getXForm().R, ccp.localAnchor1.sub(b1.getLocalCenter()));
    			Vec2 r2 = Mat22.mul(b2.getXForm().R, ccp.localAnchor2.sub(b2.getLocalCenter()));
    			
    			//Vec2 p1 = b1.m_sweep.c + r1;
    			//Vec2 p2 = b2.m_sweep.c + r2;
    			//Vec2 dp = p2 - p1;
    			float dpx = b2.m_sweep.c.x + r2.x - b1.m_sweep.c.x - r1.x;
    			float dpy = b2.m_sweep.c.y + r2.y - b1.m_sweep.c.y - r1.y;
    			

    			// Approximate the current separation.
    			float separation = dpx*normal.x + dpy*normal.y + ccp.separation;//b2Dot(dp, normal) + ccp->separation;

    			// Track max constraint error.
    			minSeparation = MathUtils.min(minSeparation, separation);

    			// Prevent large corrections and allow slop.
    			float C = baumgarte * MathUtils.clamp(separation + Settings.linearSlop, -Settings.maxLinearCorrection, 0.0f);

    			// Compute normal impulse
    			float dImpulse = -ccp.equalizedMass * C;

    			// b2Clamp the accumulated impulse
    			float impulse0 = ccp.positionImpulse;
    			ccp.positionImpulse = MathUtils.max(impulse0 + dImpulse, 0.0f);
    			dImpulse = ccp.positionImpulse - impulse0;

    			float impulsex = dImpulse * normal.x;
    			float impulsey = dImpulse * normal.y;

    			b1.m_sweep.c.x -= invMass1 * impulsex;
    			b1.m_sweep.c.y -= invMass1 * impulsey;
    			b1.m_sweep.a -= invI1 * (r1.x*impulsey - r1.y*impulsex);//b2Cross(r1, impulse);
    			b1.synchronizeTransform();

    			b2.m_sweep.c.x += invMass2 * impulsex;
    			b2.m_sweep.c.y += invMass2 * impulsey;
    			b2.m_sweep.a += invI2 * (r2.x*impulsey - r2.y*impulsex);//b2Cross(r2, impulse);
    			b2.synchronizeTransform();
    		}
    	}

    	// We can't expect minSpeparation >= -b2_linearSlop because we don't
    	// push the separation above -b2_linearSlop.
    	return minSeparation >= -1.5f * Settings.linearSlop;
    }

}

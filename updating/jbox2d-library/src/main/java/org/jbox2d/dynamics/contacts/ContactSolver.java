package org.jbox2d.dynamics.contacts;

import java.util.Stack;

import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.structs.collision.Manifold;
import org.jbox2d.structs.collision.ManifoldPoint;
import org.jbox2d.structs.collision.WorldManifold;
import org.jbox2d.structs.dynamics.contacts.ContactConstraint;
import org.jbox2d.structs.dynamics.contacts.ContactConstraintPoint;

// updated to rev 100
// pooled locally, non-threaded
/**
 * @author Daniel
 */
public class ContactSolver {
	
	/**
	 * For each solver, this is the initial number of constraints in the array, which expands
	 * as needed.
	 */
	public static final int INITIAL_NUM_CONSTRAINTS = Settings.maxTOIContacts;
	
	/**
	 * Ensure a reasonable condition number. for the block solver
	 */
	public static final float k_maxConditionNumber = 100.0f;

	
	public ContactConstraint[] m_constraints;
	public int m_constraintCount;
	
	public ContactSolver(){
		m_constraints = new ContactConstraint[INITIAL_NUM_CONSTRAINTS];
		for(int i=0; i< m_constraints.length; i++){
			m_constraints[i] = new ContactConstraint();
		}
	}
	
	// djm pooling
	private final WorldManifold worldManifold = new WorldManifold();
	private final Vec2 tangent = new Vec2();
	private final Vec2 temp1 = new Vec2();
	private final Vec2 temp2 = new Vec2();
	
	public final void init(Contact[] contacts, int contactCount, float impulseRatio){

		m_constraintCount = contactCount;
		
		// dynamic constraint array length, because we are pooling
		if(m_constraints.length <= contactCount){
			ContactConstraint[] newConstraints = new ContactConstraint[m_constraints.length*2];
			for(int i=0; i< newConstraints.length; i++){
				if(i<m_constraints.length/2){
					newConstraints[i] = m_constraints[i];
				}else{
					newConstraints[i] = new ContactConstraint();
				}
			}
		}
		
		for (int i = 0; i < m_constraintCount; ++i){
			Contact contact = contacts[i];

			Fixture fixtureA = contact.m_fixtureA;
			Fixture fixtureB = contact.m_fixtureB;
			Shape shapeA = fixtureA.getShape();
			Shape shapeB = fixtureB.getShape();
			float radiusA = shapeA.m_radius;
			float radiusB = shapeB.m_radius;
			Body bodyA = fixtureA.getBody();
			Body bodyB = fixtureB.getBody();
			Manifold manifold = contact.getManifold();

			float friction = Settings.mixFriction(fixtureA.getFriction(), fixtureB.getFriction());
			float restitution = Settings.mixRestitution(fixtureA.getRestitution(), fixtureB.getRestitution());

			Vec2 vA = bodyA.m_linearVelocity;
			Vec2 vB = bodyB.m_linearVelocity;
			float wA = bodyA.m_angularVelocity;
			float wB = bodyB.m_angularVelocity;

			assert(manifold.pointCount > 0);

			worldManifold.initialize(manifold, bodyA.m_xf, radiusA, bodyB.m_xf, radiusB);

			ContactConstraint cc = m_constraints[i];
			cc.bodyA = bodyA;
			cc.bodyB = bodyB;
			cc.manifold = manifold; // have to set actual manifold
			cc.normal.set(worldManifold.normal);
			cc.pointCount = manifold.pointCount;
			cc.friction = friction;
			cc.restitution = restitution;

			cc.localPlaneNormal.set(manifold.localNormal);
			cc.localPoint.set(manifold.localPoint);
			cc.radius = radiusA + radiusB;
			cc.type = manifold.type;

			for (int j = 0; j < cc.pointCount; ++j){
				ManifoldPoint cp = manifold.points[j];
				ContactConstraintPoint ccp = cc.points[j];

				ccp.normalImpulse = cp.normalImpulse;
				ccp.tangentImpulse = cp.tangentImpulse;

				ccp.localPoint.set(cp.localPoint);
				
				ccp.rA.x = worldManifold.points[j].x - bodyA.m_sweep.c.x;
				ccp.rA.y = worldManifold.points[j].y - bodyA.m_sweep.c.y;

				ccp.rB.x = worldManifold.points[j].x - bodyB.m_sweep.c.x;
				ccp.rB.y = worldManifold.points[j].y - bodyB.m_sweep.c.y;
				
				float rnA = Vec2.cross(ccp.rA, cc.normal);
				float rnB = Vec2.cross(ccp.rB, cc.normal);
				rnA *= rnA;
				rnB *= rnB;

				float kNormal = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rnA + bodyB.m_invI * rnB;

				assert(kNormal > Settings.EPSILON);
				ccp.normalMass = 1.0f / kNormal;

				Vec2.crossToOut(cc.normal, 1.0f, tangent);

				float rtA = Vec2.cross(ccp.rA, tangent);
				float rtB = Vec2.cross(ccp.rB, tangent);
				rtA *= rtA;
				rtB *= rtB;

				float kTangent = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rtA + bodyB.m_invI * rtB;

				assert(kTangent > Settings.EPSILON);
				ccp.tangentMass = 1.0f /  kTangent;

				// Setup a velocity bias for restitution.
				ccp.velocityBias = 0.0f;
				Vec2.crossToOut(wB, ccp.rB, temp1);
				Vec2.crossToOut(wA, ccp.rA, temp2);
				temp1.addLocal(vB).subLocal(vA).subLocal(temp2);
				
				//float vRel = Dot(cc.normal, vB + Cross(wB, ccp.rB) - vA - Cross(wA, ccp.rA));
				float vRel = Vec2.dot(cc.normal, temp1);
				
				if (vRel < -Settings.velocityThreshold){
					ccp.velocityBias = -cc.restitution * vRel;
				}
			}

			// If we have two points, then prepare the block solver.
			if (cc.pointCount == 2){
				ContactConstraintPoint ccp1 = cc.points[0];
				ContactConstraintPoint ccp2 = cc.points[1];
				
				float invMassA = bodyA.m_invMass;
				float invIA = bodyA.m_invI;
				float invMassB = bodyB.m_invMass;
				float invIB = bodyB.m_invI;

				float rn1A = Vec2.cross(ccp1.rA, cc.normal);
				float rn1B = Vec2.cross(ccp1.rB, cc.normal);
				float rn2A = Vec2.cross(ccp2.rA, cc.normal);
				float rn2B = Vec2.cross(ccp2.rB, cc.normal);

				float k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
				float k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
				float k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;

				// Ensure a reasonable condition number.
				//final float k_maxConditionNumber = 100.0f;
				if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)){
					// K is safe to invert.
					cc.K.col1.set(k11, k12);
					cc.K.col2.set(k12, k22);
					cc.normalMass.set(cc.K);
					cc.normalMass.invertLocal();
				}
				else{
					// The constraints are redundant, just use one.
					// TODO_ERIN use deepest?
					cc.pointCount = 1;
				}
			}
		}
	}
	
	// djm pooling, and from above
	private final Vec2 P = new Vec2();
	private final Vec2 temp = new Vec2();
	
	public void warmStart(){
		// Warm start.
		for (int i = 0; i < m_constraintCount; ++i){
			ContactConstraint c = m_constraints[i];

			Body bodyA = c.bodyA;
			Body bodyB = c.bodyB;
			float invMassA = bodyA.m_invMass;
			float invIA = bodyA.m_invI;
			float invMassB = bodyB.m_invMass;
			float invIB = bodyB.m_invI;
			Vec2 normal = c.normal;
			Vec2.crossToOut(normal, 1f, tangent);

			for (int j = 0; j < c.pointCount; ++j){
				ContactConstraintPoint ccp = c.points[j];
				//Vec2 P = ccp.normalImpulse * normal + ccp.tangentImpulse * tangent;
				temp.set(normal).mulLocal(ccp.normalImpulse);
				P.set(tangent).mulLocal(ccp.tangentImpulse).addLocal(temp);
				bodyA.m_angularVelocity -= invIA * Vec2.cross(ccp.rA, P);
				temp.set(P).mulLocal(invMassA);
				bodyA.m_linearVelocity.subLocal(temp);
				bodyB.m_angularVelocity += invIB * Vec2.cross(ccp.rB, P);
				temp.set(P).mulLocal(invMassB);
				bodyB.m_linearVelocity.addLocal(temp);
			}
		}
	}
	
	// djm pooling, and from above
	
//	public final void initVelocityConstraints(TimeStep step){
//		// Warm start.
//		for (int i = 0; i < m_constraintCount; ++i){
//			ContactConstraint c = m_constraints[i];
//
//			Body bodyA = c.bodyA;
//			Body bodyB = c.bodyB;
//			float invMassA = bodyA.m_invMass;
//			float invIA = bodyA.m_invI;
//			float invMassB = bodyB.m_invMass;
//			float invIB = bodyB.m_invI;
//			Vec2 normal = c.normal;
//			Vec2.crossToOut(normal, 1.0f, tangent);
//
//			if (step.warmStarting){
//				for (int j = 0; j < c.pointCount; ++j){
//					ContactConstraintPoint ccp = c.points[j];
//					ccp.normalImpulse *= step.dtRatio;
//					ccp.tangentImpulse *= step.dtRatio;
//					//Vec2 P = ccp.normalImpulse * normal + ccp.tangentImpulse * tangent;
//					temp1.set(normal).mulLocal(ccp.normalImpulse);
//					P.set(tangent).mulLocal(ccp.tangentImpulse).addLocal(temp1);
//
//					bodyA.m_angularVelocity -= invIA * Vec2.cross(ccp.rA, P);
//					//bodyA.m_linearVelocity -= invMassA * P;
//					temp1.set(P).mulLocal(invMassA);
//					bodyA.m_linearVelocity.subLocal(temp1);
//
//					bodyB.m_angularVelocity += invIB * Vec2.cross(ccp.rB, P);
//					//bodyB.m_linearVelocity += invMassB * P;
//					temp1.set(P).mulLocal(invMassB);
//					bodyB.m_linearVelocity.addLocal(temp1);
//				}
//			}
//			else{
//				for (int j = 0; j < c.pointCount; ++j){
//					ContactConstraintPoint ccp = c.points[j];
//					ccp.normalImpulse = 0.0f;
//					ccp.tangentImpulse = 0.0f;
//				}
//			}
//		}
//	}
	
	//djm pooling from above
	private final Vec2 dv = new Vec2();
	private final Vec2 a = new Vec2();
	private final Vec2 b = new Vec2();
	private final Vec2 dv1 = new Vec2();
	private final Vec2 dv2 = new Vec2();
	private final Vec2 x = new Vec2();
	private final Vec2 d = new Vec2();
	private final Vec2 P1 = new Vec2();
	private final Vec2 P2 = new Vec2();
	
	public final void solveVelocityConstraints(){
		for (int i = 0; i < m_constraintCount; ++i){
			ContactConstraint c = m_constraints[i];
			Body bodyA = c.bodyA;
			Body bodyB = c.bodyB;
			float wA = bodyA.m_angularVelocity;
			float wB = bodyB.m_angularVelocity;
			Vec2 vA = bodyA.m_linearVelocity;
			Vec2 vB = bodyB.m_linearVelocity;
			float invMassA = bodyA.m_invMass;
			float invIA = bodyA.m_invI;
			float invMassB = bodyB.m_invMass;
			float invIB = bodyB.m_invI;
			Vec2 normal = c.normal;
			Vec2.crossToOut(normal, 1.0f, tangent);
			float friction = c.friction;

			assert(c.pointCount == 1 || c.pointCount == 2);

			// Solve tangent constraints
			for (int j = 0; j < c.pointCount; ++j){
				ContactConstraintPoint ccp = c.points[j];

				// Relative velocity at contact
				//Vec2 dv = vB + Cross(wB, ccp.rB) - vA - Cross(wA, ccp.rA);
				Vec2.crossToOut(wA, ccp.rA, temp1);
				Vec2.crossToOut(wB, ccp.rB, dv);
				dv.addLocal(vB).subLocal(vA).subLocal(temp1);

				// Compute tangent force
				float vt = Vec2.dot(dv, tangent);
				float lambda = ccp.tangentMass * (-vt);

				// Clamp the accumulated force
				float maxFriction = friction * ccp.normalImpulse;
				float newImpulse = MathUtils.clamp(ccp.tangentImpulse + lambda, -maxFriction, maxFriction);
				lambda = newImpulse - ccp.tangentImpulse;

				// Apply contact impulse
				//Vec2 P = lambda * tangent;
				P.set(tangent).mulLocal(lambda);
				
				//vA -= invMassA * P;
				temp1.set(P).mulLocal(invMassA);
				vA.subLocal(temp1);
				wA -= invIA * Vec2.cross(ccp.rA, P);

				//vB += invMassB * P;
				temp1.set(P).mulLocal(invMassB);
				vB.addLocal(temp1);
				wB += invIB * Vec2.cross(ccp.rB, P);

				ccp.tangentImpulse = newImpulse;
			}

			// Solve normal constraints
			if (c.pointCount == 1){
				ContactConstraintPoint ccp = c.points[0];

				// Relative velocity at contact
				//Vec2 dv = vB + Cross(wB, ccp.rB) - vA - Cross(wA, ccp.rA);
				Vec2.crossToOut(wA, ccp.rA, temp1);
				Vec2.crossToOut(wB, ccp.rB, dv);
				dv.addLocal(vB).subLocal(vA).subLocal(temp1);

				// Compute normal impulse
				float vn = Vec2.dot(dv, normal);
				float lambda = -ccp.normalMass * (vn - ccp.velocityBias);

				// Clamp the accumulated impulse
				float newImpulse = MathUtils.max(ccp.normalImpulse + lambda, 0.0f);
				lambda = newImpulse - ccp.normalImpulse;

				// Apply contact impulse
				P.set(normal).mulLocal(lambda);
				
				//vA -= invMassA * P;
				temp1.set(P).mulLocal(invMassA);
				vA.subLocal(temp1);
				wA -= invIA * Vec2.cross(ccp.rA, P);

				//vB += invMassB * P;
				temp1.set(P).mulLocal(invMassB);
				vB.addLocal(temp1);
				wB += invIB * Vec2.cross(ccp.rB, P);
				
				ccp.normalImpulse = newImpulse;
			}
			else
			{
				// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
				// Build the mini LCP for this contact patch
				//
				// vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
				//
				// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
				// b = vn_0 - velocityBias
				//
				// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
				// implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
				// vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
				// solution that satisfies the problem is chosen.
				// 
				// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
				// that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
				//
				// Substitute:
				// 
				// x = x' - a
				// 
				// Plug into above equation:
				//
				// vn = A * x + b
				//    = A * (x' - a) + b
				//    = A * x' + b - A * a
				//    = A * x' + b'
				// b' = b - A * a;

				ContactConstraintPoint cp1 = c.points[0];
				ContactConstraintPoint cp2 = c.points[1];

				a.set(cp1.normalImpulse, cp2.normalImpulse);
				assert(a.x >= 0.0f && a.y >= 0.0f);

				// Relative velocity at contact
				//Vec2 dv1 = vB + Cross(wB, cp1.rB) - vA - Cross(wA, cp1.rA);
				Vec2.crossToOut(wA, cp1.rA, temp1);
				Vec2.crossToOut(wB, cp1.rB, dv1);
				dv.addLocal(vB).subLocal(vA).subLocal(temp1);
				
				//Vec2 dv2 = vB + Cross(wB, cp2.rB) - vA - Cross(wA, cp2.rA);
				Vec2.crossToOut(wA, cp2.rA, temp1);
				Vec2.crossToOut(wB, cp2.rB, dv2);
				dv.addLocal(vB).subLocal(vA).subLocal(temp1);
				
				// Compute normal velocity
				float vn1 = Vec2.dot(dv1, normal);
				float vn2 = Vec2.dot(dv2, normal);

				b.x = vn1 - cp1.velocityBias;
				b.y = vn2 - cp2.velocityBias;
				//b -= Mul(c.K, a);
				Mat22.mulToOut(c.K, a, temp2);
				b.subLocal(temp2);

				//final float k_errorTol = 1e-3f;
				//B2_NOT_USED(k_errorTol);

				for (;;)
				{
					//
					// Case 1: vn = 0
					//
					// 0 = A * x' + b'
					//
					// Solve for x':
					//
					// x' = - inv(A) * b'
					//
					//Vec2 x = - Mul(c.normalMass, b);
					Mat22.mulToOut(c.normalMass, b, x);
					x.mulLocal(-1);
					
					if (x.x >= 0.0f && x.y >= 0.0f){
						// Resubstitute for the incremental impulse
						//Vec2 d = x - a;
						d.set(x).subLocal(a);
						
						// Apply incremental impulse
						//Vec2 P1 = d.x * normal;
						//Vec2 P2 = d.y * normal;
						P1.set(normal).mulLocal(d.x);
						P2.set(normal).mulLocal(d.y);
						
						/*vA -= invMassA * (P1 + P2);
						wA -= invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));

						vB += invMassB * (P1 + P2);
						wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));*/
						
						temp1.set(P1).addLocal(P2);
						temp2.set(temp1).mulLocal(invMassA);
						vA.subLocal(temp2);
						temp2.set(temp1).mulLocal(invMassB);
						vB.addLocal(temp2);
												
						wA -= invIA * (Vec2.cross(cp1.rA, P1) + Vec2.cross(cp2.rA, P2));
						wB += invIB * (Vec2.cross(cp1.rB, P1) + Vec2.cross(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

	/*#if B2_DEBUG_SOLVER == 1
						// Postconditions
						dv1 = vB + Cross(wB, cp1.rB) - vA - Cross(wA, cp1.rA);
						dv2 = vB + Cross(wB, cp2.rB) - vA - Cross(wA, cp2.rA);

						// Compute normal velocity
						vn1 = Dot(dv1, normal);
						vn2 = Dot(dv2, normal);

						assert(Abs(vn1 - cp1.velocityBias) < k_errorTol);
						assert(Abs(vn2 - cp2.velocityBias) < k_errorTol);
	#endif*/
						break;
					}

					//
					// Case 2: vn1 = 0 and x2 = 0
					//
					//   0 = a11 * x1' + a12 * 0 + b1' 
					// vn2 = a21 * x1' + a22 * 0 + '
					//
					x.x = - cp1.normalMass * b.x;
					x.y = 0.0f;
					vn1 = 0.0f;
					vn2 = c.K.col1.y * x.x + b.y;

					if (x.x >= 0.0f && vn2 >= 0.0f)
					{
						// Resubstitute for the incremental impulse
						d.set(x).subLocal(a);

						// Apply incremental impulse
						//Vec2 P1 = d.x * normal;
						//Vec2 P2 = d.y * normal;
						P1.set(normal).mulLocal(d.x);
						P2.set(normal).mulLocal(d.y);
						
						/*Vec2 P1 = d.x * normal;
						Vec2 P2 = d.y * normal;
						vA -= invMassA * (P1 + P2);
						wA -= invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));

						vB += invMassB * (P1 + P2);
						wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));*/
						
						temp1.set(P1).addLocal(P2);
						temp2.set(temp1).mulLocal(invMassA);
						vA.subLocal(temp2);
						temp2.set(temp1).mulLocal(invMassB);
						vB.addLocal(temp2);
												
						wA -= invIA * (Vec2.cross(cp1.rA, P1) + Vec2.cross(cp2.rA, P2));
						wB += invIB * (Vec2.cross(cp1.rB, P1) + Vec2.cross(cp2.rB, P2));
						
						
						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

	/*#if B2_DEBUG_SOLVER == 1
						// Postconditions
						dv1 = vB + Cross(wB, cp1.rB) - vA - Cross(wA, cp1.rA);

						// Compute normal velocity
						vn1 = Dot(dv1, normal);

						assert(Abs(vn1 - cp1.velocityBias) < k_errorTol);
	#endif*/
						break;
					}


					//
					// Case 3: wB = 0 and x1 = 0
					//
					// vn1 = a11 * 0 + a12 * x2' + b1' 
					//   0 = a21 * 0 + a22 * x2' + '
					//
					x.x = 0.0f;
					x.y = - cp2.normalMass * b.y;
					vn1 = c.K.col2.x * x.y + b.x;
					vn2 = 0.0f;

					if (x.y >= 0.0f && vn1 >= 0.0f)
					{
						// Resubstitute for the incremental impulse
						d.set(x).subLocal(a);

						// Apply incremental impulse
						/*Vec2 P1 = d.x * normal;
						Vec2 P2 = d.y * normal;
						vA -= invMassA * (P1 + P2);
						wA -= invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));

						vB += invMassB * (P1 + P2);
						wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));*/
						
						P1.set(normal).mulLocal(d.x);
						P2.set(normal).mulLocal(d.y);
						
						temp1.set(P1).addLocal(P2);
						temp2.set(temp1).mulLocal(invMassA);
						vA.subLocal(temp2);
						temp2.set(temp1).mulLocal(invMassB);
						vB.addLocal(temp2);
												
						wA -= invIA * (Vec2.cross(cp1.rA, P1) + Vec2.cross(cp2.rA, P2));
						wB += invIB * (Vec2.cross(cp1.rB, P1) + Vec2.cross(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

	/*#if B2_DEBUG_SOLVER == 1
						// Postconditions
						dv2 = vB + Cross(wB, cp2.rB) - vA - Cross(wA, cp2.rA);

						// Compute normal velocity
						vn2 = Dot(dv2, normal);

						assert(Abs(vn2 - cp2.velocityBias) < k_errorTol);
	#endif*/
						break;
					}

					//
					// Case 4: x1 = 0 and x2 = 0
					// 
					// vn1 = b1
					// vn2 = ;
					x.x = 0.0f;
					x.y = 0.0f;
					vn1 = b.x;
					vn2 = b.y;

					if (vn1 >= 0.0f && vn2 >= 0.0f )
					{
						// Resubstitute for the incremental impulse
						d.set(x).subLocal(a);

						// Apply incremental impulse
						/*Vec2 P1 = d.x * normal;
						Vec2 P2 = d.y * normal;
						vA -= invMassA * (P1 + P2);
						wA -= invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));

						vB += invMassB * (P1 + P2);
						wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));*/

						P1.set(normal).mulLocal(d.x);
						P2.set(normal).mulLocal(d.y);
						
						temp1.set(P1).addLocal(P2);
						temp2.set(temp1).mulLocal(invMassA);
						vA.subLocal(temp2);
						temp2.set(temp1).mulLocal(invMassB);
						vB.addLocal(temp2);
												
						wA -= invIA * (Vec2.cross(cp1.rA, P1) + Vec2.cross(cp2.rA, P2));
						wB += invIB * (Vec2.cross(cp1.rB, P1) + Vec2.cross(cp2.rB, P2));
						
						
						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

						break;
					}

					// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
					break;
				}
			}

			bodyA.m_linearVelocity.set(vA);
			bodyA.m_angularVelocity = wA;
			bodyB.m_linearVelocity.set(vB);
			bodyB.m_angularVelocity = wB;
		}
	}
	
	public void storeImpulses(){
		for( int i=0; i<m_constraintCount; i++){
			ContactConstraint c = m_constraints[i];
			Manifold m = c.manifold;
			
			for(int j=0; j< c.pointCount; j++){
				m.points[j].normalImpulse = c.points[j].normalImpulse;
				m.points[j].tangentImpulse = c.points[j].tangentImpulse;
			}
		}
	}
	
	public final void finalizeVelocityConstraints(){
		for (int i = 0; i < m_constraintCount; ++i){
			ContactConstraint c = m_constraints[i];
			Manifold m = c.manifold;

			for (int j = 0; j < c.pointCount; ++j){
				m.points[j].normalImpulse = c.points[j].normalImpulse;
				m.points[j].tangentImpulse = c.points[j].tangentImpulse;
			}
		}
	}

	/*#if 0
	// Sequential solver.
	bool ContactSolver::SolvePositionConstraints(float baumgarte)
	{
		float minSeparation = 0.0f;
	
		for (int i = 0; i < m_constraintCount; ++i)
		{
			ContactConstraint* c = m_constraints + i;
			Body* bodyA = c.bodyA;
			Body* bodyB = c.bodyB;
			float invMassA = bodyA.m_mass * bodyA.m_invMass;
			float invIA = bodyA.m_mass * bodyA.m_invI;
			float invMassB = bodyB.m_mass * bodyB.m_invMass;
			float invIB = bodyB.m_mass * bodyB.m_invI;
	
			Vec2 normal = c.normal;
	
			// Solve normal constraints
			for (int j = 0; j < c.pointCount; ++j)
			{
				ContactConstraintPoint* ccp = c.points + j;
	
				Vec2 r1 = Mul(bodyA.GetXForm().R, ccp.localAnchorA - bodyA.GetLocalCenter());
				Vec2 r2 = Mul(bodyB.GetXForm().R, ccp.localAnchorB - bodyB.GetLocalCenter());
	
				Vec2 p1 = bodyA.m_sweep.c + r1;
				Vec2 p2 = bodyB.m_sweep.c + r2;
				Vec2 dp = p2 - p1;
	
				// Approximate the current separation.
				float separation = Dot(dp, normal) + ccp.separation;
	
				// Track max constraint error.
				minSeparation = Min(minSeparation, separation);
	
				// Prevent large corrections and allow slop.
				float C = Clamp(baumgarte * (separation + _linearSlop), -_maxLinearCorrection, 0.0f);
	
				// Compute normal impulse
				float impulse = -ccp.equalizedMass * C;
	
				Vec2 P = impulse * normal;
	
				bodyA.m_sweep.c -= invMassA * P;
				bodyA.m_sweep.a -= invIA * Cross(r1, P);
				bodyA.SynchronizeTransform();
	
				bodyB.m_sweep.c += invMassB * P;
				bodyB.m_sweep.a += invIB * Cross(r2, P);
				bodyB.SynchronizeTransform();
			}
		}
	
		// We can't expect minSpeparation >= -_linearSlop because we don't
		// push the separation above -_linearSlop.
		return minSeparation >= -1.5f * _linearSlop;
	}*/
	
	// djm pooling, and from above
	private final Stack<PositionSolverManifold> solverStack = new Stack<PositionSolverManifold>();
	private final Vec2 rA = new Vec2();
	private final Vec2 rB = new Vec2();
	
	/**
	 * Sequential solver.
	 */
	public final boolean solvePositionConstraints(float baumgarte){
		float minSeparation = 0.0f;

		for (int i = 0; i < m_constraintCount; ++i){
			ContactConstraint c = m_constraints[i];
			Body bodyA = c.bodyA;
			Body bodyB = c.bodyB;

			float invMassA = bodyA.m_mass * bodyA.m_invMass;
			float invIA = bodyA.m_mass * bodyA.m_invI;
			float invMassB = bodyB.m_mass * bodyB.m_invMass;
			float invIB = bodyB.m_mass * bodyB.m_invI;

			// Solve normal constraints
			for (int j = 0; j < c.pointCount; ++j){
				if(solverStack.isEmpty()){
					solverStack.push(new PositionSolverManifold());
					solverStack.push(new PositionSolverManifold());
					solverStack.push(new PositionSolverManifold());
				}
				PositionSolverManifold psm = solverStack.pop();
				psm.initialize(c, j);
				Vec2 normal = psm.normal;
				
				Vec2 point = psm.point;
				float separation = psm.separation;

				rA.set(point).subLocal(bodyA.m_sweep.c);
				rB.set(point).subLocal(bodyB.m_sweep.c);

				// Track max constraint error.
				minSeparation = MathUtils.min(minSeparation, separation);

				// Prevent large corrections and allow slop.
				float C = MathUtils.clamp(baumgarte * (separation + Settings.linearSlop), Settings.maxLinearCorrection, 0.0f);

				// Compute the effective mass.
				float rnA = Vec2.cross(rA, normal);
				float rnB = Vec2.cross(rB, normal);
				float K = invMassA + invMassB + invIA * rnA * rnA + invIB * rnB * rnB;
				
				// Compute normal impulse
				float impulse = K > 0.0f ? - C / K : 0.0f;

				P.set(normal).mulLocal(impulse);

				temp1.set(P).mulLocal(invMassA);
				bodyA.m_sweep.c.subLocal(temp1);;
				bodyA.m_sweep.a -= invIA * Vec2.cross(rA, P);
				bodyA.synchronizeTransform();

				temp1.set(P).mulLocal(invMassB);
				bodyB.m_sweep.c.addLocal(temp1);
				bodyB.m_sweep.a += invIB * Vec2.cross(rB, P);
				bodyB.synchronizeTransform();
			}
		}

		// We can't expect minSpeparation >= -linearSlop because we don't
		// push the separation above -linearSlop.
		return minSeparation >= -1.5f * Settings.linearSlop;
	}
}

class PositionSolverManifold{
	
	public final Vec2 normal = new Vec2();
	public final Vec2 point = new Vec2();
	public float separation;
	
	// djm pooling
	private final Vec2 pointA = new Vec2();
	private final Vec2 pointB = new Vec2();
	private final Vec2 temp = new Vec2();
	private final Vec2 planePoint = new Vec2();
	private final Vec2 clipPoint = new Vec2();
	
	public void initialize(ContactConstraint cc, int index){
		assert(cc.pointCount > 0);

		switch (cc.type){
			case CIRCLES:{
				cc.bodyA.getWorldPointToOut(cc.localPoint, pointA);
				cc.bodyB.getWorldPointToOut(cc.points[0].localPoint, pointB);
				if (MathUtils.distanceSquared(pointA, pointB) > Settings.EPSILON * Settings.EPSILON){
					normal.set(pointB).subLocal(pointA);
					normal.normalize();
				}
				else{
					normal.set(1.0f, 0.0f);
				}

				point.set(pointA).addLocal(pointB).mulLocal(.5f);
				temp.set(pointB).subLocal(pointA);
				separation = Vec2.dot(temp, normal) - cc.radius;
				break;
			}
	
			case FACE_A:{
				cc.bodyA.getWorldVectorToOut(cc.localPlaneNormal, normal);
				cc.bodyA.getWorldPointToOut(cc.localPoint, planePoint);

				cc.bodyB.getWorldPointToOut(cc.points[index].localPoint, clipPoint);
				temp.set(clipPoint).subLocal(planePoint);
				separation = Vec2.dot(temp, normal) - cc.radius;
				point.set(clipPoint);
				break;
			}
	
			case FACE_B:
				{
					cc.bodyB.getWorldPointToOut(cc.localPoint, planePoint);
					cc.bodyB.getWorldPointToOut(cc.localPoint, planePoint);
	
					cc.bodyA.getWorldPointToOut(cc.points[index].localPoint, clipPoint);
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

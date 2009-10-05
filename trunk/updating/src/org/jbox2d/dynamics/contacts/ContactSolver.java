package org.jbox2d.dynamics.contacts;

import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.pooling.TLWorldManifold;
import org.jbox2d.structs.collision.Manifold;
import org.jbox2d.structs.collision.WorldManifold;
import org.jbox2d.structs.dynamics.contacts.ContactConstraint;

public class ContactSolver {
	
	public TimeStep m_step;
	public ContactConstraint[] m_constraints;
	public int m_constraintCount;
	
	public ContactSolver(){
		
	}
	
	public static final TLWorldManifold tlworldManifold = new TLWorldManifold();
	
	public final void init(TimeStep step, Contact[] contacts, int contactCount){
		m_step = step;

		m_constraintCount = contactCount;
		m_constraints = new ContactConstraint[contactCount];

		for (int i = 0; i < m_constraintCount; ++i)
		{
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

			assert(manifold.m_pointCount > 0);

			WorldManifold worldManifold = tlworldManifold.get();
			worldManifold.initialize(manifold, bodyA.m_xf, radiusA, bodyB.m_xf, radiusB);

			ContactConstraint cc = m_constraints[i];
			cc.bodyA = bodyA;
			cc.bodyB = bodyB;
			cc.manifold = manifold;
			cc.normal = worldManifold.m_normal;
			cc.pointCount = manifold.m_pointCount;
			cc.friction = friction;
			cc.restitution = restitution;

			cc.localPlaneNormal = manifold.m_localPlaneNormal;
			cc.localPoint = manifold.m_localPoint;
			cc.radius = radiusA + radiusB;
			cc.type = manifold.m_type;

			for (int j = 0; j < cc.pointCount; ++j)
			{
				ManifoldPoint cp = manifold.m_points + j;
				ContactConstraintPoint ccp = cc.points + j;

				ccp.normalImpulse = cp.m_normalImpulse;
				ccp.tangentImpulse = cp.m_tangentImpulse;

				ccp.localPoint = cp.m_localPoint;

				ccp.rA = worldManifold.m_points[j] - bodyA.m_sweep.c;
				ccp.rB = worldManifold.m_points[j] - bodyB.m_sweep.c;

				float rnA = Cross(ccp.rA, cc.normal);
				float rnB = Cross(ccp.rB, cc.normal);
				rnA *= rnA;
				rnB *= rnB;

				float kNormal = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rnA + bodyB.m_invI * rnB;

				Assert(kNormal > B2_FLT_EPSILON);
				ccp.normalMass = 1.0f / kNormal;

				float kEqualized = bodyA.m_mass * bodyA.m_invMass + bodyB.m_mass * bodyB.m_invMass;
				kEqualized += bodyA.m_mass * bodyA.m_invI * rnA + bodyB.m_mass * bodyB.m_invI * rnB;

				Assert(kEqualized > B2_FLT_EPSILON);
				ccp.equalizedMass = 1.0f / kEqualized;

				Vec2 tangent = Cross(cc.normal, 1.0f);

				float rtA = Cross(ccp.rA, tangent);
				float rtB = Cross(ccp.rB, tangent);
				rtA *= rtA;
				rtB *= rtB;

				float kTangent = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rtA + bodyB.m_invI * rtB;

				Assert(kTangent > B2_FLT_EPSILON);
				ccp.tangentMass = 1.0f /  kTangent;

				// Setup a velocity bias for restitution.
				ccp.velocityBias = 0.0f;
				float vRel = Dot(cc.normal, vB + Cross(wB, ccp.rB) - vA - Cross(wA, ccp.rA));
				if (vRel < -_velocityThreshold)
				{
					ccp.velocityBias = -cc.restitution * vRel;
				}
			}

			// If we have two points, then prepare the block solver.
			if (cc.pointCount == 2)
			{
				ContactConstraintPoint ccp1 = cc.points + 0;
				ContactConstraintPoint ccp2 = cc.points + 1;
				
				float invMassA = bodyA.m_invMass;
				float invIA = bodyA.m_invI;
				float invMassB = bodyB.m_invMass;
				float invIB = bodyB.m_invI;

				float rn1A = Cross(ccp1.rA, cc.normal);
				float rn1B = Cross(ccp1.rB, cc.normal);
				float rn2A = Cross(ccp2.rA, cc.normal);
				float rn2B = Cross(ccp2.rB, cc.normal);

				float k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
				float k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
				float k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;

				// Ensure a reasonable condition number.
				const float k_maxConditionNumber = 100.0f;
				if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
				{
					// K is safe to invert.
					cc.K.col1.Set(k11, k12);
					cc.K.col2.Set(k12, k22);
					cc.normalMass = cc.K.getInverse();
				}
				else
				{
					// The constraints are redundant, just use one.
					// TODO_ERIN use deepest?
					cc.pointCount = 1;
				}
			}
		}
	}
	
	public final void initVelocityConstraints(TimeStep step){
		
	}
	
	public final void solveVelocityConstraints(){
		
	}
	public final void finalizeVelocityConstraints(){
		
	}

	public final boolean solvePositionConstraints(float baumgarte){
		
	}
}

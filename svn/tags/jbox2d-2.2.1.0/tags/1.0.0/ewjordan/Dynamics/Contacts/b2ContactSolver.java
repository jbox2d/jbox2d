/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "b2ContactSolver.h"
#include "b2Contact.h"
#include "../../Dynamics/b2Body.h"
#include "../../Common/b2StackAllocator.h"

b2ContactSolver::b2ContactSolver(b2Contact** contacts, int32 contactCount, b2StackAllocator* allocator)
{
	m_allocator = allocator;

	m_constraintCount = 0;
	for (int32 i = 0; i < contactCount; ++i)
	{
		m_constraintCount += contacts[i]->GetManifoldCount();
	}

	m_constraints = (b2ContactConstraint*)m_allocator->Allocate(m_constraintCount * sizeof(b2ContactConstraint));

	int32 count = 0;
	for (int32 i = 0; i < contactCount; ++i)
	{
		b2Contact* contact = contacts[i];
		b2Body* b1 = contact->m_shape1->m_body;
		b2Body* b2 = contact->m_shape2->m_body;
		int32 manifoldCount = contact->GetManifoldCount();
		b2Manifold* manifolds = contact->GetManifolds();
		float32 friction = contact->m_friction;
		float32 restitution = contact->m_restitution;

		b2Vec2 v1 = b1->m_linearVelocity;
		b2Vec2 v2 = b2->m_linearVelocity;
		float32 w1 = b1->m_angularVelocity;
		float32 w2 = b2->m_angularVelocity;

		for (int32 j = 0; j < manifoldCount; ++j)
		{
			b2Manifold* manifold = manifolds + j;

			b2Assert(manifold->pointCount > 0);

			const b2Vec2 normal = manifold->normal;

			b2Assert(count < m_constraintCount);
			b2ContactConstraint* c = m_constraints + count;
			c->body1 = b1;
			c->body2 = b2;
			c->manifold = manifold;
			c->normal = normal;
			c->pointCount = manifold->pointCount;
			c->friction = friction;
			c->restitution = restitution;

			for (int32 k = 0; k < c->pointCount; ++k)
			{
				b2ContactPoint* cp = manifold->points + k;
				b2ContactConstraintPoint* ccp = c->points + k;

				ccp->normalImpulse = cp->normalImpulse;
				ccp->tangentImpulse = cp->tangentImpulse;
				ccp->separation = cp->separation;

				b2Vec2 r1 = cp->position - b1->m_position;
				b2Vec2 r2 = cp->position - b2->m_position;

				ccp->localAnchor1 = b2MulT(b1->m_R, r1);
				ccp->localAnchor2 = b2MulT(b2->m_R, r2);

				float32 r1Sqr = b2Dot(r1, r1);
				float32 r2Sqr = b2Dot(r2, r2);

				float32 rn1 = b2Dot(r1, normal);
				float32 rn2 = b2Dot(r2, normal);
				float32 kNormal = b1->m_invMass + b2->m_invMass;
				kNormal += b1->m_invI * (r1Sqr - rn1 * rn1) + b2->m_invI * (r2Sqr - rn2 * rn2);
				b2Assert(kNormal > FLT_EPSILON);
				ccp->normalMass = 1.0f / kNormal;

				b2Vec2 tangent = b2Cross(normal, 1.0f);

				float32 rt1 = b2Dot(r1, tangent);
				float32 rt2 = b2Dot(r2, tangent);
				float32 kTangent = b1->m_invMass + b2->m_invMass;
				kTangent += b1->m_invI * (r1Sqr - rt1 * rt1) + b2->m_invI * (r2Sqr - rt2 * rt2);
				b2Assert(kTangent > FLT_EPSILON);
				ccp->tangentMass = 1.0f /  kTangent;

				// Setup a velocity bias for restitution.
				float32 vRel = b2Dot(c->normal, v2 + b2Cross(w2, r2) - v1 - b2Cross(w1, r1));
				if (vRel < -b2_velocityThreshold)
				{
					ccp->velocityBias = -c->restitution * vRel;
				}
				else
				{
					ccp->velocityBias = 0.0f;
				}
			}

			++count;
		}
	}

	b2Assert(count == m_constraintCount);
}

b2ContactSolver::~b2ContactSolver()
{
	m_allocator->Free(m_constraints);
}

void b2ContactSolver::PreSolve()
{
	// Warm start.
	for (int32 i = 0; i < m_constraintCount; ++i)
	{
		b2ContactConstraint* c = m_constraints + i;

		b2Body* b1 = c->body1;
		b2Body* b2 = c->body2;
		float32 invMass1 = b1->m_invMass;
		float32 invI1 = b1->m_invI;
		float32 invMass2 = b2->m_invMass;
		float32 invI2 = b2->m_invI;
		b2Vec2 normal = c->normal;
		b2Vec2 tangent = b2Cross(normal, 1.0f);

		for (int32 j = 0; j < c->pointCount; ++j)
		{
			b2ContactConstraintPoint* ccp = c->points + j;
			b2Vec2 P = ccp->normalImpulse * normal + ccp->tangentImpulse * tangent;
			b2Vec2 r1 = b2Mul(b1->m_R, ccp->localAnchor1);
			b2Vec2 r2 = b2Mul(b2->m_R, ccp->localAnchor2);
			b1->m_angularVelocity -= invI1 * b2Cross(r1, P);
			b1->m_linearVelocity -= invMass1 * P;
			b2->m_angularVelocity += invI2 * b2Cross(r2, P);
			b2->m_linearVelocity += invMass2 * P;

			ccp->positionImpulse = 0.0f;
		}
	}
}

void b2ContactSolver::SolveVelocityConstraints()
{
	for (int32 i = 0; i < m_constraintCount; ++i)
	{
		b2ContactConstraint* c = m_constraints + i;
		b2Body* b1 = c->body1;
		b2Body* b2 = c->body2;
		float32 invMass1 = b1->m_invMass;
		float32 invI1 = b1->m_invI;
		float32 invMass2 = b2->m_invMass;
		float32 invI2 = b2->m_invI;
		b2Vec2 normal = c->normal;
		b2Vec2 tangent = b2Cross(normal, 1.0f);

		// Solver normal constraints
		for (int32 j = 0; j < c->pointCount; ++j)
		{
			b2ContactConstraintPoint* ccp = c->points + j;

			b2Vec2 r1 = b2Mul(b1->m_R, ccp->localAnchor1);
			b2Vec2 r2 = b2Mul(b2->m_R, ccp->localAnchor2);

			// Relative velocity at contact
			b2Vec2 dv = b2->m_linearVelocity + b2Cross(b2->m_angularVelocity, r2) - b1->m_linearVelocity - b2Cross(b1->m_angularVelocity, r1);

			// Compute normal impulse
			float32 vn = b2Dot(dv, normal);
			float32 lambda = -ccp->normalMass * (vn - ccp->velocityBias);

			// b2Clamp the accumulated impulse
			float32 newImpulse = b2Max(ccp->normalImpulse + lambda, 0.0f);
			lambda = newImpulse - ccp->normalImpulse;

			// Apply contact impulse
			b2Vec2 P = lambda * normal;

			b1->m_linearVelocity -= invMass1 * P;
			b1->m_angularVelocity -= invI1 * b2Cross(r1, P);

			b2->m_linearVelocity += invMass2 * P;
			b2->m_angularVelocity += invI2 * b2Cross(r2, P);

			ccp->normalImpulse = newImpulse;
		}

		// Solver tangent constraints
		for (int32 j = 0; j < c->pointCount; ++j)
		{
			b2ContactConstraintPoint* ccp = c->points + j;

			b2Vec2 r1 = b2Mul(b1->m_R, ccp->localAnchor1);
			b2Vec2 r2 = b2Mul(b2->m_R, ccp->localAnchor2);

			// Relative velocity at contact
			b2Vec2 dv = b2->m_linearVelocity + b2Cross(b2->m_angularVelocity, r2) - b1->m_linearVelocity - b2Cross(b1->m_angularVelocity, r1);

			// Compute tangent impulse
			float32 vt = b2Dot(dv, tangent);
			float32 lambda = ccp->tangentMass * (-vt);

			// b2Clamp the accumulated impulse
			float32 maxFriction = c->friction * ccp->normalImpulse;
			float32 newImpulse = b2Clamp(ccp->tangentImpulse + lambda, -maxFriction, maxFriction);
			lambda = newImpulse - ccp->tangentImpulse;

			// Apply contact impulse
			b2Vec2 P = lambda * tangent;

			b1->m_linearVelocity -= invMass1 * P;
			b1->m_angularVelocity -= invI1 * b2Cross(r1, P);

			b2->m_linearVelocity += invMass2 * P;
			b2->m_angularVelocity += invI2 * b2Cross(r2, P);

			ccp->tangentImpulse = newImpulse;
		}
	}
}

bool b2ContactSolver::SolvePositionConstraints(float32 beta)
{
	float32 minSeparation = 0.0f;

	for (int32 i = 0; i < m_constraintCount; ++i)
	{
		b2ContactConstraint* c = m_constraints + i;
		b2Body* b1 = c->body1;
		b2Body* b2 = c->body2;
		float32 invMass1 = b1->m_invMass;
		float32 invI1 = b1->m_invI;
		float32 invMass2 = b2->m_invMass;
		float32 invI2 = b2->m_invI;
		b2Vec2 normal = c->normal;
		b2Vec2 tangent = b2Cross(normal, 1.0f);

		// Solver normal constraints
		for (int32 j = 0; j < c->pointCount; ++j)
		{
			b2ContactConstraintPoint* ccp = c->points + j;

			b2Vec2 r1 = b2Mul(b1->m_R, ccp->localAnchor1);
			b2Vec2 r2 = b2Mul(b2->m_R, ccp->localAnchor2);

			b2Vec2 p1 = b1->m_position + r1;
			b2Vec2 p2 = b2->m_position + r2;
			b2Vec2 dp = p2 - p1;

			// Approximate the current separation.
			float32 separation = b2Dot(dp, normal) + ccp->separation;

			// Prevent large corrections
			separation = b2Max(separation, -b2_maxLinearCorrection);

			// Track max constraint error.
			minSeparation = b2Min(minSeparation, separation);

			// Compute normal impulse
			float32 dImpulse = -ccp->normalMass * beta * b2Min(0.0f, separation + b2_linearSlop);

			// b2Clamp the accumulated impulse
			float32 impulse0 = ccp->positionImpulse;
			ccp->positionImpulse = b2Max(impulse0 + dImpulse, 0.0f);
			dImpulse = ccp->positionImpulse - impulse0;

			b2Vec2 impulse = dImpulse * normal;

			b1->m_position -= invMass1 * impulse;
			b1->m_rotation -= invI1 * b2Cross(r1, impulse);
			b1->m_R.Set(b1->m_rotation);

			b2->m_position += invMass2 * impulse;
			b2->m_rotation += invI2 * b2Cross(r2, impulse);
			b2->m_R.Set(b2->m_rotation);
		}
	}

	return minSeparation >= -b2_linearSlop;
}

void b2ContactSolver::PostSolve()
{
	for (int32 i = 0; i < m_constraintCount; ++i)
	{
		b2ContactConstraint* c = m_constraints + i;
		b2Manifold* m = c->manifold;

		for (int32 j = 0; j < c->pointCount; ++j)
		{
			m->points[j].normalImpulse = c->points[j].normalImpulse;
			m->points[j].tangentImpulse = c->points[j].tangentImpulse;
		}
	}
}/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef CONTACT_SOLVER_H
#define CONTACT_SOLVER_H

#include "../../Common/b2Math.h"
#include "../../Collision/b2Collision.h"

struct b2Contact;
struct b2Body;
struct b2Island;
class b2StackAllocator;

struct b2ContactConstraintPoint
{
	b2Vec2 localAnchor1;
	b2Vec2 localAnchor2;
	float32 normalImpulse;
	float32 tangentImpulse;
	float32 positionImpulse;
	float32 normalMass;
	float32 tangentMass;
	float32 separation;
	float32 velocityBias;
};

struct b2ContactConstraint
{
	b2ContactConstraintPoint points[b2_maxManifoldPoints];
	b2Vec2 normal;
	b2Manifold* manifold;
	b2Body* body1;
	b2Body* body2;
	float32 friction;
	float32 restitution;
	int32 pointCount;
};

struct b2ContactSolver
{
	b2ContactSolver(b2Contact** contacts, int32 contactCount, b2StackAllocator* allocator);
	~b2ContactSolver();

	void PreSolve();
	void SolveVelocityConstraints();
	bool SolvePositionConstraints(float32 beta);
	void PostSolve();

	b2StackAllocator* m_allocator;
	b2ContactConstraint* m_constraints;
	int m_constraintCount;
};

#endif

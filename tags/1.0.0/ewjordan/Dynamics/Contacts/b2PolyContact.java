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

#include "b2PolyContact.h"
#include "../../Common/b2BlockAllocator.h"

#include <string.h>
#include <new.h>

b2Contact* b2PolyContact::Create(b2Shape* shape1, b2Shape* shape2, b2BlockAllocator* allocator)
{
	void* mem = allocator->Allocate(sizeof(b2PolyContact));
	return new (mem) b2PolyContact(shape1, shape2);
}

void b2PolyContact::Destroy(b2Contact* contact, b2BlockAllocator* allocator)
{
	((b2PolyContact*)contact)->~b2PolyContact();
	allocator->Free(contact, sizeof(b2PolyContact));
}

b2PolyContact::b2PolyContact(b2Shape* s1, b2Shape* s2)
	: b2Contact(s1, s2)
{
	b2Assert(m_shape1->m_type == e_polyShape);
	b2Assert(m_shape2->m_type == e_polyShape);
	m_manifold.pointCount = 0;
}

void b2PolyContact::Evaluate()
{
	b2Manifold m0;
	memcpy(&m0, &m_manifold, sizeof(b2Manifold));

	b2CollidePoly(&m_manifold, (b2PolyShape*)m_shape1, (b2PolyShape*)m_shape2);

	// Match contact ids to facilitate warm starting.
	if (m_manifold.pointCount > 0)
	{
		bool match[b2_maxManifoldPoints] = {false, false};

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for (int32 i = 0; i < m_manifold.pointCount; ++i)
		{
			b2ContactPoint* cp = m_manifold.points + i;
			cp->normalImpulse = 0.0f;
			cp->tangentImpulse = 0.0f;
			b2ContactID id = cp->id;

			for (int32 j = 0; j < m0.pointCount; ++j)
			{
				if (match[j] == true)
					continue;

				b2ContactPoint* cp0 = m0.points + j;
				b2ContactID id0 = cp0->id;

				if (id0.key == id.key)
				{
					match[j] = true;
					m_manifold.points[i].normalImpulse = m0.points[j].normalImpulse;
					m_manifold.points[i].tangentImpulse = m0.points[j].tangentImpulse;
					break;
				}
			}
		}

		m_manifoldCount = 1;
	}
	else
	{
		m_manifoldCount = 0;
	}
}
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

#ifndef POLYCONTACT_H
#define POLYCONTACT_H

#include "b2Contact.h"

class b2BlockAllocator;

struct b2PolyContact : public b2Contact
{
	static b2Contact* Create(b2Shape* shape1, b2Shape* shape2, b2BlockAllocator* allocator);
	static void Destroy(b2Contact* contact, b2BlockAllocator* allocator);

	b2PolyContact(b2Shape* shape1, b2Shape* shape2);
	~b2PolyContact() {}

	void Evaluate();
	b2Manifold* GetManifolds()
	{
		return &m_manifold;
	}

	b2Manifold m_manifold;
};

#endif

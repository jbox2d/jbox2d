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

#include "b2PolyAndCircleContact.h"
#include "../../Common/b2BlockAllocator.h"

#include <new.h>

b2Contact* b2PolyAndCircleContact::Create(b2Shape* shape1, b2Shape* shape2, b2BlockAllocator* allocator)
{
	void* mem = allocator->Allocate(sizeof(b2PolyAndCircleContact));
	return new (mem) b2PolyAndCircleContact(shape1, shape2);
}

void b2PolyAndCircleContact::Destroy(b2Contact* contact, b2BlockAllocator* allocator)
{
	((b2PolyAndCircleContact*)contact)->~b2PolyAndCircleContact();
	allocator->Free(contact, sizeof(b2PolyAndCircleContact));
}

b2PolyAndCircleContact::b2PolyAndCircleContact(b2Shape* s1, b2Shape* s2)
: b2Contact(s1, s2)
{
	b2Assert(m_shape1->m_type == e_polyShape);
	b2Assert(m_shape2->m_type == e_circleShape);
	m_manifold.pointCount = 0;
	m_manifold.points[0].normalImpulse = 0.0f;
	m_manifold.points[0].tangentImpulse = 0.0f;
}

void b2PolyAndCircleContact::Evaluate()
{
	b2CollidePolyAndCircle(&m_manifold, (b2PolyShape*)m_shape1, (b2CircleShape*)m_shape2);

	if (m_manifold.pointCount > 0)
	{
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

#ifndef POLY_AND_CIRCLE_CONTACT_H
#define POLY_AND_CIRCLE_CONTACT_H

#include "b2Contact.h"

class b2BlockAllocator;

struct b2PolyAndCircleContact : public b2Contact
{
	static b2Contact* Create(b2Shape* shape1, b2Shape* shape2, b2BlockAllocator* allocator);
	static void Destroy(b2Contact* contact, b2BlockAllocator* allocator);

	b2PolyAndCircleContact(b2Shape* shape1, b2Shape* shape2);
	~b2PolyAndCircleContact() {}

	void Evaluate();
	b2Manifold* GetManifolds()
	{
		return &m_manifold;
	}

	b2Manifold m_manifold;
};

#endif

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

import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.collision.shapes.ShapeType;
import org.jbox2d.common.MathUtils;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.ContactListener;
import org.jbox2d.dynamics.World;

// Updated to rev 142 of b2Contact.h/cpp

/**
 * Base class for contacts between shapes.
 * @author ewjordan
 *
 */
public abstract class Contact {

	public static final int e_nonSolidFlag	= 0x0001;
	public static final int e_slowFlag		= 0x0002;
	public static final int e_islandFlag	= 0x0004;
	public static final int e_toiFlag		= 0x0008;

	static ArrayList<ContactRegister> s_registers;

	static boolean s_initialized;

	/** The parent world. */
	public World m_world;

	/* World pool and list pointers. */
	public Contact m_prev;
	public Contact m_next;

	/** Node for connecting bodies. */
	public final ContactEdge m_node1;
	/** Node for connecting bodies. */
	public final ContactEdge m_node2;

	public Shape m_shape1;
	public Shape m_shape2;

	/** Combined friction */
	public float m_friction;
	/** Combined restitution */
	public float m_restitution;

	// public boolean m_islandFlag;
	public int m_flags;
	public int m_manifoldCount;

	public float m_toi;

	public abstract void evaluate(ContactListener listener);

	/** Get the manifold array. */
	public abstract List<Manifold> getManifolds();

	/**
	 * Get the number of manifolds. This is 0 or 1 between convex shapes.
	 * This may be greater than 1 for convex-vs-concave shapes. Each
	 * manifold holds up to two contact points with a shared contact normal.
	 */
	public int getManifoldCount() {
		/*
		 * List<Manifold> m = GetManifolds(); if (m == null) return 0; else
		 * return GetManifolds().size();
		 */
		return m_manifoldCount;
	}

	public boolean isSolid() {
		return (m_flags & e_nonSolidFlag) == 0;
	}



	public Contact() {
		m_node1 = new ContactEdge();
		m_node2 = new ContactEdge();
	}

	public Contact(final Shape s1, final Shape s2) {
		this();

		m_flags = 0;

		if (s1.isSensor() || s2.isSensor()) {
			m_flags |= e_nonSolidFlag;
		}

		m_shape1 = s1;
		m_shape2 = s2;

		m_manifoldCount = 0;
		//getManifolds().clear(); //unnecessary, I think// djm now causes error

		m_friction = MathUtils.sqrt(m_shape1.m_friction * m_shape2.m_friction);
		m_restitution = MathUtils.max(m_shape1.m_restitution, m_shape2.m_restitution);
		//m_world = s1.m_body.m_world;
		m_prev = null;
		m_next = null;
		m_node1.contact = null;
		m_node1.prev = null;
		m_node1.next = null;
		m_node1.other = null;

		m_node2.contact = null;
		m_node2.prev = null;
		m_node2.next = null;
		m_node2.other = null;
	}

	public Contact getNext() {
		return m_next;
	}

	public Shape getShape1() {
		return m_shape1;
	}

	public Shape getShape2() {
		return m_shape2;
	}


	public void update(final ContactListener listener) {
		final int oldCount = getManifoldCount();
		evaluate(listener);
		final int newCount = getManifoldCount();

		final Body body1 = m_shape1.getBody();
		final Body body2 = m_shape2.getBody();

		if (newCount == 0 && oldCount > 0) {
			body1.wakeUp();
			body2.wakeUp();
		}

		// Slow contacts don't generate TOI events.
		if (body1.isStatic() || body1.isBullet() || body2.isStatic() || body2.isBullet()) {
			m_flags &= ~e_slowFlag;
		} else {
			m_flags |= e_slowFlag;
		}
	}

	/**
	 * returns a clone of this contact.  rev 166: not used in the engine
	 */
	@Override
	public abstract Contact clone();

	public static final void initializeRegisters() {
		s_registers = new ArrayList<ContactRegister>();
		Contact.addType(new CircleContact(), ShapeType.CIRCLE_SHAPE,
		                ShapeType.CIRCLE_SHAPE);
		Contact.addType(new PolyAndCircleContact(), ShapeType.POLYGON_SHAPE,
		                ShapeType.CIRCLE_SHAPE);
		Contact.addType(new PolyContact(), ShapeType.POLYGON_SHAPE,
		                ShapeType.POLYGON_SHAPE);
		Contact.addType(new PolyAndEdgeContact(), ShapeType.POLYGON_SHAPE,
		                ShapeType.EDGE_SHAPE);
		Contact.addType(new EdgeAndCircleContact(), ShapeType.EDGE_SHAPE,
		                ShapeType.CIRCLE_SHAPE);
		Contact.addType(new PointAndCircleContact(), ShapeType.POINT_SHAPE,
		                ShapeType.CIRCLE_SHAPE);
		Contact.addType(new PointAndPolyContact(), ShapeType.POLYGON_SHAPE,
		                ShapeType.POINT_SHAPE);
	}

	public static final void addType(final ContactCreateFcn createFcn, final ShapeType type1,
	                                 final ShapeType type2) {
		final ContactRegister cr = new ContactRegister();
		cr.s1 = type1;
		cr.s2 = type2;
		cr.createFcn = createFcn;
		cr.primary = true;
		s_registers.add(cr);

		if (type1 != type2) {
			final ContactRegister cr2 = new ContactRegister();
			cr2.s2 = type1;
			cr2.s1 = type2;
			cr2.createFcn = createFcn;
			cr2.primary = false;
			s_registers.add(cr2);
		}
	}

	/* Java note:
	 * This function is called "create" in C++ version.
	 * Doing this in Java causes problems, so leave it as is.
	 */
	public static final Contact createContact(final Shape shape1, final Shape shape2) {
		if (s_initialized == false) {
			Contact.initializeRegisters();
			s_initialized = true;
		}

		final ShapeType type1 = shape1.m_type;
		final ShapeType type2 = shape2.m_type;

		// assert ShapeType.UNKNOWN_SHAPE< type1 && type1 <
		// ShapeType.SHAPE_TYPE_COUNT;
		// assert ShapeType.UNKNOWN_SHAPE < type2 && type2 <
		// ShapeType.SHAPE_TYPE_COUNT;
		final ContactRegister register = Contact.getContactRegister(type1, type2);
		if (register != null) {
			if (register.primary) {
				return register.createFcn.create(shape1, shape2);
			} else {
				final Contact c = register.createFcn.create(shape2, shape1);
				for (int i = 0; i < c.getManifoldCount(); ++i) {
					final Manifold m = c.getManifolds().get(i);
					m.normal.negateLocal();
				}
				return c;
			}
		} else {
			return null;
		}
	}

	private static final ContactRegister getContactRegister(final ShapeType type1,
	                                                        final ShapeType type2) {
		for (int i=0; i<s_registers.size(); ++i) {//ContactRegister cr : s_registers) {
			final ContactRegister cr = s_registers.get(i);
			if (cr.s1 == type1 && cr.s2 == type2) {
				return cr;
			}
		}

		return null;
	}

	public static final void destroy(final Contact contact) {
		assert (s_initialized == true);

		if (contact.getManifoldCount() > 0) {
			contact.getShape1().getBody().wakeUp();
			contact.getShape2().getBody().wakeUp();
		}
	}
}

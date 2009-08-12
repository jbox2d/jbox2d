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
import org.jbox2d.collision.ManifoldPoint;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.collision.shapes.ShapeType;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.ContactListener;
import org.jbox2d.pooling.SingletonPool;
import org.jbox2d.pooling.TLContactPoint;
import org.jbox2d.pooling.TLManifold;
import org.jbox2d.pooling.TLVec2;

public class EdgeAndCircleContact extends Contact implements ContactCreateFcn {
	public final Manifold m_manifold;
	public final ArrayList<Manifold> manifoldList = new ArrayList<Manifold>();

	public EdgeAndCircleContact() {
		// TODO Auto-generated constructor stub
		super();
		m_manifold = new Manifold();
		manifoldList.add(m_manifold);
		m_manifoldCount = 0;
	}

	public EdgeAndCircleContact(final Shape s1, final Shape s2) {
		super(s1, s2);
		assert(m_shape1.getType() == ShapeType.EDGE_SHAPE);
		assert(m_shape2.getType() == ShapeType.CIRCLE_SHAPE);
		m_manifold = new Manifold();
		manifoldList.add(m_manifold);
		m_manifoldCount = 0;
	}

	@Override
	public Contact clone() {
		assert false: "Not yet implemented.";
	return this;
	}

	public static void Destroy(final Contact contact) {
		((EdgeAndCircleContact) contact).destructor();
	}

	public void destructor() {

	}

	private static final TLManifold tlm0 = new TLManifold();
	private static final TLVec2 tlV1 = new TLVec2();
	private static final TLContactPoint tlCp = new TLContactPoint();
	@Override
	public void evaluate(final ContactListener listener) {
		final Body b1 = m_shape1.getBody();
		final Body b2 = m_shape2.getBody();

		
		final Manifold m0 = tlm0.get();
		final Vec2 v1 = tlV1.get();
		final ContactPoint cp = tlCp.get();
		
		m0.set(m_manifold);
		
		SingletonPool.getCollideCircle().collideEdgeAndCircle(m_manifold, (EdgeShape)m_shape1, b1.getMemberXForm(), (CircleShape)m_shape2, b2.getMemberXForm());

		cp.shape1 = m_shape1;
		cp.shape2 = m_shape2;
		//TODO
		cp.friction = m_friction;
		cp.restitution = m_restitution;
		//cp.friction = b2MixFriction(m_shape1->GetFriction(), m_shape2->GetFriction());
		//cp.restitution = b2MixRestitution(m_shape1->GetRestitution(), m_shape2->GetRestitution());

		if (m_manifold.pointCount > 0) {
			m_manifoldCount = 1;
			final ManifoldPoint mp = m_manifold.points[0];

			if (m0.pointCount == 0) {
				mp.normalImpulse = 0.0f;
				mp.tangentImpulse = 0.0f;

				if (listener != null) {
					b1.getWorldLocationToOut(mp.localPoint1, cp.position);
					//Vec2 v1 = b1.getLinearVelocityFromLocalPoint(mp.localPoint1);
					b1.getLinearVelocityFromLocalPointToOut(mp.localPoint1, v1);
					//Vec2 v2 = b2.getLinearVelocityFromLocalPoint(mp.localPoint2);
					b2.getLinearVelocityFromLocalPointToOut(mp.localPoint2, cp.velocity);
					//cp.velocity = v2.sub(v1);
					cp.velocity.subLocal(v1);

					cp.normal.set(m_manifold.normal);
					cp.separation = mp.separation;
					cp.id.set(mp.id);
					listener.add(cp);
				}
			} else {
				final ManifoldPoint mp0 = m0.points[0];
				mp.normalImpulse = mp0.normalImpulse;
				mp.tangentImpulse = mp0.tangentImpulse;

				if (listener != null) {
					b1.getWorldLocationToOut(mp.localPoint1, cp.position);
					//cp.position = b1.getWorldLocation(mp.localPoint1);
					//Vec2 v1 = b1.getLinearVelocityFromLocalPoint(mp.localPoint1);
					b1.getLinearVelocityFromLocalPointToOut(mp.localPoint1, v1);
					//Vec2 v2 = b2.getLinearVelocityFromLocalPoint(mp.localPoint2);
					b2.getLinearVelocityFromLocalPointToOut(mp.localPoint2, cp.velocity);
					//cp.velocity = v2.sub(v1);
					cp.velocity.subLocal(v1);

					cp.normal.set(m_manifold.normal);
					cp.separation = mp.separation;
					cp.id.set(mp.id);
					listener.persist(cp);
				}
			}
		} else {
			m_manifoldCount = 0;
			if (m0.pointCount > 0 && (listener != null)) {
				final ManifoldPoint mp0 = m0.points[0];
				b1.getWorldLocationToOut(mp0.localPoint1, cp.position);
				//cp.position = b1.getWorldLocation(mp0.localPoint1);
				//Vec2 v1 = b1.getLinearVelocityFromLocalPoint(mp.localPoint1);
				b1.getLinearVelocityFromLocalPointToOut(mp0.localPoint1, v1);
				//Vec2 v2 = b2.getLinearVelocityFromLocalPoint(mp.localPoint2);
				b2.getLinearVelocityFromLocalPointToOut(mp0.localPoint2, cp.velocity);
				//cp.velocity = v2.sub(v1);
				cp.velocity.subLocal(v1);

				cp.normal.set(m_manifold.normal);
				cp.separation = mp0.separation;
				cp.id.set(mp0.id);
				listener.remove(cp);
			}
		}
	}

	@Override
	public List<Manifold> getManifolds() {
		return manifoldList;
	}

	public Contact create(final Shape s1, final Shape s2) {

		return new EdgeAndCircleContact(s1,s2);
	}

}

package org.jbox2d.dynamics.contacts;

import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.pooling.SingletonPool;
import org.jbox2d.structs.collision.ShapeType;
import org.jbox2d.structs.dynamics.contacts.ContactCreateFcn;

public class PolygonContact extends Contact implements ContactCreateFcn {

	public PolygonContact(Fixture fixtureA, Fixture fixtureB) {
		super(fixtureA, fixtureB);
		assert(m_fixtureA.getType() == ShapeType.POLYGON_SHAPE);
		assert(m_fixtureB.getType() == ShapeType.POLYGON_SHAPE);
	}

	@Override
	protected void evaluate() {
		Body bodyA = m_fixtureA.getBody();
		Body bodyB = m_fixtureB.getBody();
		
		SingletonPool.getCollision().collidePolygons(m_manifold,
				m_fixtureA.getShape(), bodyA.getTransform(),
				m_fixtureB.getShape(), bodyB.getTransform());
	}

	@Override
	public Contact contactCreateFcn(Fixture fixtureA, Fixture fixtureB) {
		// TODO djm: pool these
		return new PolygonContact(fixtureA, fixtureB);
	}

	@Override
	public void contactDestroyFcn(Contact contact) {
		// TODO djm: pooling
	}

}

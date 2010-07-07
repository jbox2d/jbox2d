package org.jbox2d.dynamics.contacts;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Transform;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.pooling.SingletonPool;
import org.jbox2d.structs.collision.Manifold;
import org.jbox2d.structs.collision.shapes.ShapeType;

// updated to rev 100

public class PolygonContact extends Contact {

	public PolygonContact(){}
	
	public PolygonContact(Fixture fixtureA, Fixture fixtureB) {
		super(fixtureA, fixtureB);
		assert(m_fixtureA.getType() == ShapeType.POLYGON);
		assert(m_fixtureB.getType() == ShapeType.POLYGON);
	}

	@Override
	public void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
		SingletonPool.getCollision().collidePolygons(m_manifold,
				(PolygonShape)m_fixtureA.getShape(), xfA,
				(PolygonShape)m_fixtureB.getShape(), xfB);
	}
}

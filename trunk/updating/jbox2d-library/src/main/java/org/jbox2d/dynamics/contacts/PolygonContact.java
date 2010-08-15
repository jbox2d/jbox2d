package org.jbox2d.dynamics.contacts;

import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.ShapeType;
import org.jbox2d.common.Transform;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.pooling.SingletonPool;

// updated to rev 100

public class PolygonContact extends Contact {
	
	public PolygonContact(){}
	
	public void init(Fixture fixtureA, Fixture fixtureB) {
		super.init(fixtureA, fixtureB);
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

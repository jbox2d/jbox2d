package org.jbox2d.dynamics.contacts;

import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.ShapeType;
import org.jbox2d.common.Transform;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.pooling.SingletonPool;

// updated to rev 100

public class PolygonAndCircleContact extends Contact {
	
	public PolygonAndCircleContact(){}
	
	
	public void init(Fixture fixtureA, Fixture fixtureB){
		super.init(fixtureA, fixtureB);
		assert(m_fixtureA.getType() == ShapeType.POLYGON);
		assert(m_fixtureB.getType() == ShapeType.CIRCLE);
	}

	@Override
	public void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
		SingletonPool.getCollision().collidePolygonAndCircle(m_manifold,
				(PolygonShape)m_fixtureA.getShape(), xfA,
				(CircleShape)m_fixtureB.getShape(), xfB);
	}
}
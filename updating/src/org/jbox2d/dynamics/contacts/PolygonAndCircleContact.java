package org.jbox2d.dynamics.contacts;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.pooling.SingletonPool;
import org.jbox2d.structs.collision.shapes.ShapeType;
import org.jbox2d.structs.dynamics.contacts.ContactCreateFcn;

public class PolygonAndCircleContact extends Contact implements ContactCreateFcn {
	
	public PolygonAndCircleContact(){}
	
	public PolygonAndCircleContact(Fixture fixtureA, Fixture fixtureB) {
		super(fixtureA, fixtureB);
		assert(m_fixtureA.getType() == org.jbox2d.structs.collision.shapes.POLYGON_SHAPE);
		assert(m_fixtureB.getType() == org.jbox2d.structs.collision.shapes.CIRCLE_SHAPE);
	}

	@Override
	protected void evaluate() {
		Body bodyA = m_fixtureA.getBody();
		Body bodyB = m_fixtureB.getBody();
		
		SingletonPool.getCollision().collidePolygonAndCircle(m_manifold,
				(PolygonShape)m_fixtureA.getShape(), bodyA.getTransform(),
				(CircleShape)m_fixtureB.getShape(), bodyB.getTransform());
	}

	@Override
	public Contact contactCreateFcn(Fixture fixtureA, Fixture fixtureB) {
		// TODO djm: pool these
		return new PolygonAndCircleContact(fixtureA, fixtureB);
	}

	@Override
	public void contactDestroyFcn(Contact contact) {
		// TODO djm: pooling
	}

}
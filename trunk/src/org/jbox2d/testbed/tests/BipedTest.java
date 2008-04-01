package org.jbox2d.testbed.tests;

import org.jbox2d.collision.CircleDef;
import org.jbox2d.collision.PolygonDef;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;

public class BipedTest extends AbstractExample {
	private boolean firstTime = true;
	Biped m_biped;
	
	public BipedTest(TestbedMain p) {
		super(p);
	}

	@Override
	public void create() {
		if (firstTime) {
			setCamera(0f, 20f, 15f);
			firstTime = false;
		}
		
		final float k_restitution = 1.4f;
		settings.drawJoints = false;

		{
			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, 20.0f);
			Body body = m_world.createStaticBody(bd);

			PolygonDef sd = new PolygonDef();
			sd.density = 0.0f;
			sd.restitution = k_restitution;

			sd.setAsBox(0.1f, 10.0f, new Vec2(-10.0f, 0.0f), 0.0f);
			body.createShape(sd);

			sd.setAsBox(0.1f, 10.0f, new Vec2(10.0f, 0.0f), 0.0f);
			body.createShape(sd);

			sd.setAsBox(0.1f, 10.0f, new Vec2(0.0f, -10.0f), 0.5f * (float)Math.PI);
			body.createShape(sd);

			sd.setAsBox(0.1f, 10.0f, new Vec2(0.0f, 10.0f), -0.5f * (float)Math.PI);
			body.createShape(sd);
		}

		m_biped = new Biped(m_world, new Vec2(0.0f, 20.0f));

		for (int i = 0; i < 8; ++i) {
			BodyDef bd = new BodyDef();
			bd.position.set(5.0f, 20.0f + i);
			bd.isBullet = true;
			Body body = m_world.createDynamicBody(bd);
			body.setLinearVelocity(new Vec2(0.0f, -100.0f));
			body.setAngularVelocity(parent.random(-50.0f, 50.0f));

			CircleDef sd = new CircleDef();
			sd.radius = 0.25f;
			sd.density = 15.0f;
			sd.restitution = k_restitution;
			body.createShape(sd);
			body.setMassFromShapes();
		}
	}

	@Override
	public String getName() {
		return "Biped Test";
	}

}

package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.testbed.framework.TestbedTest;

public class VaryingFriction extends TestbedTest {

	@Override
	public String getTestName() {
		return "Varying Friction";
	}

	@Override
	public void initTest() {
		{
			BodyDef bd = new BodyDef();
			Body ground = world.createBody(bd);

			PolygonShape shape = new PolygonShape();
			shape.setAsEdge(new Vec2(-40.0f, 0.0f),new Vec2(40.0f, 0.0f));
			ground.createFixture(shape, 0.0f);
		}

		{
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(13.0f, 0.25f);

			BodyDef bd = new BodyDef();
			bd.position.set(-4.0f, 22.0f);
			bd.angle = -0.25f;

			Body ground = world.createBody(bd);
			ground.createFixture(shape, 0.0f);
		}

		{
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(0.25f, 1.0f);

			BodyDef bd = new BodyDef();
			bd.position.set(10.5f, 19.0f);

			Body ground = world.createBody(bd);
			ground.createFixture(shape, 0.0f);
		}

		{
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(13.0f, 0.25f);

			BodyDef bd = new BodyDef();
			bd.position.set(4.0f, 14.0f);
			bd.angle = 0.25f;

			Body ground = world.createBody(bd);
			ground.createFixture(shape, 0.0f);
		}

		{
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(0.25f, 1.0f);

			BodyDef bd = new BodyDef();
			bd.position.set(-10.5f, 11.0f);

			Body ground = world.createBody(bd);
			ground.createFixture(shape, 0.0f);
		}

		{
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(13.0f, 0.25f);

			BodyDef bd = new BodyDef();
			bd.position.set(-4.0f, 6.0f);
			bd.angle = -0.25f;

			Body ground = world.createBody(bd);
			ground.createFixture(shape, 0.0f);
		}

		{
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(0.5f, 0.5f);

			FixtureDef fd = new FixtureDef();
			fd.shape = shape;
			fd.density = 25.0f;

			float friction[] = {0.75f, 0.5f, 0.35f, 0.1f, 0.0f};

			for (int i = 0; i < 5; ++i)
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType.DYNAMIC;
				bd.position.set(-15.0f + 4.0f * i, 28.0f);
				Body body = world.createBody(bd);

				fd.friction = friction[i];
				body.createFixture(fd);
			}
		}
	}

}

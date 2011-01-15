/**
 * Created at 4:11:55 AM Jan 15, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.WeldJointDef;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class Cantilever extends TestbedTest {
	
	Body m_middle;
	
	int e_count = 8;
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#initTest()
	 */
	@Override
	public void initTest() {
		Body ground = null;
		{
			BodyDef bd = new BodyDef();
			ground = world.createBody(bd);

			PolygonShape shape = new PolygonShape();
			shape.setAsEdge(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
			ground.createFixture(shape, 0.0f);
		}

		{
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(0.5f, 0.125f);

			FixtureDef fd = new FixtureDef();
			fd.shape = shape;
			fd.density = 20.0f;

			WeldJointDef jd = new WeldJointDef();

			Body prevBody = ground;
			for (int i = 0; i < e_count; ++i)
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType.DYNAMIC;
				bd.position.set(-14.5f + 1.0f * i, 5.0f);
				Body body = world.createBody(bd);
				body.createFixture(fd);

				Vec2 anchor = new Vec2(-15.0f + 1.0f * i, 5.0f);
				jd.initialize(prevBody, body, anchor);
				world.createJoint(jd);

				prevBody = body;
			}
		}

		{
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(0.5f, 0.125f);

			FixtureDef fd = new FixtureDef();
			fd.shape = shape;
			fd.density = 20.0f;

			WeldJointDef jd = new WeldJointDef();

			Body prevBody = ground;
			for (int i = 0; i < e_count; ++i)
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType.DYNAMIC;
				bd.position.set(-14.5f + 1.0f * i, 15.0f);
				bd.inertiaScale = 10.0f;
				Body body = world.createBody(bd);
				body.createFixture(fd);

				Vec2 anchor = new Vec2(-15.0f + 1.0f * i, 15.0f);
				jd.initialize(prevBody, body, anchor);
				world.createJoint(jd);

				prevBody = body;
			}
		}

		{
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(0.5f, 0.125f);

			FixtureDef fd = new FixtureDef();
			fd.shape = shape;
			fd.density = 20.0f;

			WeldJointDef jd = new WeldJointDef();

			Body prevBody = ground;
			for (int i = 0; i < e_count; ++i)
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType.DYNAMIC;
				bd.position.set(-4.5f + 1.0f * i, 5.0f);
				Body body = world.createBody(bd);
				body.createFixture(fd);

				if (i > 0)
				{
					Vec2 anchor = new Vec2(-5.0f + 1.0f * i, 5.0f);
					jd.initialize(prevBody, body, anchor);
					world.createJoint(jd);
				}

				prevBody = body;
			}
		}

		{
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(0.5f, 0.125f);

			FixtureDef fd = new FixtureDef();
			fd.shape = shape;
			fd.density = 20.0f;

			WeldJointDef jd = new WeldJointDef();

			Body prevBody = ground;
			for (int i = 0; i < e_count; ++i)
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType.DYNAMIC;
				bd.position.set(5.5f + 1.0f * i, 10.0f);
				bd.inertiaScale = 10.0f;
				Body body = world.createBody(bd);
				body.createFixture(fd);

				if (i > 0)
				{
					Vec2 anchor = new Vec2(5.0f + 1.0f * i, 10.0f);
					jd.initialize(prevBody, body, anchor);
					world.createJoint(jd);
				}

				prevBody = body;
			}
		}

		for (int i = 0; i < 2; ++i)
		{
			Vec2 vertices[] = new Vec2[3];
			vertices[0] = new Vec2(-0.5f, 0.0f);
			vertices[1] = new Vec2(0.5f, 0.0f);
			vertices[2] = new Vec2(0.0f, 1.5f);

			PolygonShape shape = new PolygonShape();
			shape.set(vertices, 3);

			FixtureDef fd = new FixtureDef();
			fd.shape = shape;
			fd.density = 1.0f;

			BodyDef bd = new BodyDef();
			bd.type = BodyType.DYNAMIC;
			bd.position.set(-8.0f + 8.0f * i, 12.0f);
			Body body = world.createBody(bd);
			body.createFixture(fd);
		}

		for (int i = 0; i < 2; ++i)
		{
			CircleShape shape = new CircleShape();
			shape.m_radius = 0.5f;

			FixtureDef fd = new FixtureDef();
			fd.shape = shape;
			fd.density = 1.0f;

			BodyDef bd = new BodyDef();
			bd.type = BodyType.DYNAMIC;
			bd.position.set(-6.0f + 6.0f * i, 10.0f);
			Body body = world.createBody(bd);
			body.createFixture(fd);
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "Cantilever";
	}
	
}

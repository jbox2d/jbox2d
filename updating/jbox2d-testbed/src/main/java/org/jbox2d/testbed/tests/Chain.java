/**
 * Created at 5:30:15 AM Jan 14, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class Chain extends TestbedTest {
	
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
			shape.setAsBox(0.6f, 0.125f);

			FixtureDef fd = new FixtureDef();
			fd.shape = shape;
			fd.density = 20.0f;
			fd.friction = 0.2f;

			RevoluteJointDef jd = new RevoluteJointDef();
			jd.collideConnected = false;

			final float y = 25.0f;
			Body prevBody = ground;
			for (int i = 0; i < 30; ++i)
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType.DYNAMIC;
				bd.position.set(0.5f + i, y);
				Body body = world.createBody(bd);
				body.createFixture(fd);

				Vec2 anchor = new Vec2(i, y);
				jd.initialize(prevBody, body, anchor);
				world.createJoint(jd);

				prevBody = body;
			}
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "Chain";
	}
	
}

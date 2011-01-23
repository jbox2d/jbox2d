/**
 * Created at 1:59:32 PM Jan 23, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class VaryingRestitution extends TestbedTest{

	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#initTest()
	 */
	@Override
	public void initTest() {
		{
			BodyDef bd = new BodyDef();
			Body ground = m_world.createBody(bd);

			PolygonShape shape = new PolygonShape();
			shape.setAsEdge(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
			ground.createFixture(shape, 0.0f);
		}

		{
			CircleShape shape = new CircleShape();
			shape.m_radius = 1.0f;

			FixtureDef fd = new FixtureDef();
			fd.shape = shape;
			fd.density = 1.0f;

			float restitution[] = {0.0f, 0.1f, 0.3f, 0.5f, 0.75f, 0.9f, 1.0f};

			for (int i = 0; i < 7; ++i)
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType.DYNAMIC;
				bd.position.set(-10.0f + 3.0f * i, 20.0f);

				Body body = m_world.createBody(bd);

				fd.restitution = restitution[i];
				body.createFixture(fd);
			}
		}
	}

	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "Varying Restitution";
	}

}

/**
 * Created at 4:31:15 AM Jan 15, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class SphereStack extends TestbedTest {
	
	int e_count = 10;
	Body m_bodies[] = new Body[e_count];
	
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

			for (int i = 0; i < e_count; ++i)
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType.DYNAMIC;
				bd.position.set(0.0f, 4.0f + 3.0f * i);

				m_bodies[i] = m_world.createBody(bd);

				m_bodies[i].createFixture(shape, 1.0f);

				//m_bodies[i].setLinearVelocity(new Vec2(0.0f, -100.0f));
			}
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "Sphere Stack";
	}
	
}

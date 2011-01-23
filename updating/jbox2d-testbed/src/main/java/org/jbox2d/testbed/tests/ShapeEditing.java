/**
 * Created at 2:04:52 PM Jan 23, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class ShapeEditing extends TestbedTest {
	
	Body m_body;
	Fixture m_fixture1;
	Fixture m_fixture2;
	
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

		BodyDef bd = new BodyDef();
		bd.type = BodyType.DYNAMIC;
		bd.position.set(0.0f, 10.0f);
		m_body = m_world.createBody(bd);

		PolygonShape shape = new PolygonShape();
		shape.setAsBox(4.0f, 4.0f, new Vec2(0.0f, 0.0f), 0.0f);
		m_fixture1 = m_body.createFixture(shape, 10.0f);

		m_fixture2 = null;
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#keyPressed(char, int)
	 */
	@Override
	public void keyPressed(char key, int argKeyCode) {
		switch (key)
		{
		case 'c':
			if (m_fixture2 == null)
			{
				CircleShape shape = new CircleShape();
				shape.m_radius = 3.0f;
				shape.m_p.set(0.5f, -4.0f);
				m_fixture2 = m_body.createFixture(shape, 10.0f);
				m_body.setAwake(true);
			}
			break;

		case 'd':
			if (m_fixture2 != null)
			{
				m_body.destroyFixture(m_fixture2);
				m_fixture2 = null;
				m_body.setAwake(true);
			}
			break;
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#step(org.jbox2d.testbed.framework.TestbedSettings)
	 */
	@Override
	public void step(TestbedSettings settings) {
		super.step(settings);
		addTextLine( "Press: (c) create a shape, (d) destroy a shape.");
	}
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "Shape Editing";
	}
	
}

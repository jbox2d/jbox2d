/**
 * Created at 12:12:33 AM Jan 22, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.joints.LineJointDef;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class LineJointTest extends TestbedTest {
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#initTest()
	 */
	@Override
	public void initTest() {
		Body ground = null;
		{
			PolygonShape shape = new PolygonShape();
			shape.setAsEdge(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));

			BodyDef bd = new BodyDef();
			ground = m_world.createBody(bd);
			ground.createFixture(shape, 0.0f);
		}

		{
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(0.5f, 2.0f);

			BodyDef bd = new BodyDef();
			bd.type = BodyType.DYNAMIC;
			bd.position.set(0.0f, 7.0f);
			Body body = m_world.createBody(bd);
			body.createFixture(shape, 1.0f);

			LineJointDef jd = new LineJointDef();
			Vec2 axis = new Vec2(2.0f, 1.0f);
			axis.normalize();
			jd.initialize(ground, body, new Vec2(0.0f, 8.5f), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorForce = 100.0f;
			jd.enableMotor = true;
			jd.lowerTranslation = -4.0f;
			jd.upperTranslation = 4.0f;
			jd.enableLimit = true;
			m_world.createJoint(jd);
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "Line Joint";
	}
	
}

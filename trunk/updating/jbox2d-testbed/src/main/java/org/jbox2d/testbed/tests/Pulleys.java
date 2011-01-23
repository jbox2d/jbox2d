/**
 * Created at 12:46:04 PM Jan 23, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.joints.PulleyJoint;
import org.jbox2d.dynamics.joints.PulleyJointDef;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class Pulleys extends TestbedTest {
	
	PulleyJoint m_joint1;
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#initTest()
	 */
	@Override
	public void initTest() {
		Body ground = null;
		{
			BodyDef bd = new BodyDef();
			ground = m_world.createBody(bd);

			PolygonShape shape = new PolygonShape();
			shape.setAsEdge(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
			ground.createFixture(shape, 0.0f);
		}

		{
			float a = 2.0f;
			float b = 4.0f;
			float y = 16.0f;
			float L = 12.0f;

			PolygonShape shape = new PolygonShape();
			shape.setAsBox(a, b);

			BodyDef bd = new BodyDef();
			bd.type = BodyType.DYNAMIC;

			bd.position.set(-10.0f, y);
			Body body1 = m_world.createBody(bd);
			body1.createFixture(shape, 5.0f);

			bd.position.set(10.0f, y);
			Body body2 = m_world.createBody(bd);
			body2.createFixture(shape, 5.0f);

			PulleyJointDef pulleyDef = new PulleyJointDef();
			Vec2 anchor1 = new Vec2(-10.0f, y + b);
			Vec2 anchor2 = new Vec2(10.0f, y + b);
			Vec2 groundAnchor1 = new Vec2(-10.0f, y + b + L);
			Vec2 groundAnchor2 = new Vec2(10.0f, y + b + L);
			pulleyDef.initialize(body1, body2, groundAnchor1, groundAnchor2, anchor1, anchor2, 2.0f);

			m_joint1 = (PulleyJoint)m_world.createJoint(pulleyDef);
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#step(org.jbox2d.testbed.framework.TestbedSettings)
	 */
	@Override
	public void step(TestbedSettings settings) {
		super.step(settings);
		float ratio = m_joint1.getRatio();
		float L = m_joint1.getLength1() + ratio * m_joint1.getLength2();
		addTextLine("L1 + "+ratio+" * L2 = "+L);
		if(L >= 36){
			addTextLine("Pulley is taught");
		}
	}
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "Pulleys";
	}
	
}

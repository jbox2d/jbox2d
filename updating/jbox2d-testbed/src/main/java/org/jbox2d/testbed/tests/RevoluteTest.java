/**
 * Created at 7:59:38 PM Jan 12, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.framework.TestPanel;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class RevoluteTest extends TestbedTest {
	
	private RevoluteJoint m_joint;
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
			CircleShape shape = new CircleShape();
			shape.m_radius = 0.5f;

			BodyDef bd = new BodyDef();
			bd.type = BodyType.DYNAMIC;

			RevoluteJointDef rjd = new RevoluteJointDef();

			bd.position.set(0.0f, 20.0f);
			Body body = world.createBody(bd);
			body.createFixture(shape, 5.0f);

			float w = 100.0f;
			body.setAngularVelocity(w);
			body.setLinearVelocity(new Vec2(-8.0f * w, 0.0f));

			rjd.initialize(ground, body, new Vec2(0.0f, 12.0f));
			rjd.motorSpeed = 1.0f * MathUtils.PI;
			rjd.maxMotorTorque = 10000.0f;
			rjd.enableMotor = false;
			rjd.lowerAngle = -0.25f * MathUtils.PI;
			rjd.upperAngle = 0.5f * MathUtils.PI;
			rjd.enableLimit = true;
			rjd.collideConnected = true;

			m_joint = (RevoluteJoint)world.createJoint(rjd);
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#step(org.jbox2d.testbed.framework.TestbedSettings)
	 */
	@Override
	public void step(TestbedSettings settings) {
		super.step(settings);
		addTextLine("Limits = "+m_joint.isLimitEnabled()+", Motor = "+m_joint.isMotorEnabled());
		addTextLine("Keys: (l) limits, (a) left, (s) off, (d) right");
		
		if(TestPanel.keys['l']){
			m_joint.enableLimit(!m_joint.isLimitEnabled());
			TestPanel.keys['l']= false;
		}
		else if(TestPanel.keys['s']){
			m_joint.enableMotor(!m_joint.isMotorEnabled());
			TestPanel.keys['s']= false;
		}
	}
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "Prismatic Test";
	}
	
}

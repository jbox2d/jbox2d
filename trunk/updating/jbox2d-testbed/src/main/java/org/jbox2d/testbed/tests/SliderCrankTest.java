/**
 * Created at 7:47:37 PM Jan 12, 2011
 */
package org.jbox2d.testbed.tests;

import java.util.Formatter;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.joints.PrismaticJoint;
import org.jbox2d.dynamics.joints.PrismaticJointDef;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.framework.TestPanel;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class SliderCrankTest extends TestbedTest {
	
	private RevoluteJoint m_joint1;
	private PrismaticJoint m_joint2;
	
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
			Body prevBody = ground;

			// Define crank.
			{
				PolygonShape shape = new PolygonShape();
				shape.setAsBox(0.5f, 2.0f);

				BodyDef bd = new BodyDef();
				bd.type = BodyType.DYNAMIC;
				bd.position.set(0.0f, 7.0f);
				Body body = world.createBody(bd);
				body.createFixture(shape, 2.0f);

				RevoluteJointDef rjd = new RevoluteJointDef();
				rjd.initialize(prevBody, body, new Vec2(0.0f, 5.0f));
				rjd.motorSpeed = 1.0f * MathUtils.PI;
				rjd.maxMotorTorque = 10000.0f;
				rjd.enableMotor = true;
				m_joint1 = (RevoluteJoint)world.createJoint(rjd);

				prevBody = body;
			}

			// Define follower.
			{
				PolygonShape shape = new PolygonShape();
				shape.setAsBox(0.5f, 4.0f);

				BodyDef bd = new BodyDef();
				bd.type = BodyType.DYNAMIC;
				bd.position.set(0.0f, 13.0f);
				Body body = world.createBody(bd);
				body.createFixture(shape, 2.0f);

				RevoluteJointDef rjd = new RevoluteJointDef();
				rjd.initialize(prevBody, body, new Vec2(0.0f, 9.0f));
				rjd.enableMotor = false;
				world.createJoint(rjd);

				prevBody = body;
			}

			// Define piston
			{
				PolygonShape shape = new PolygonShape();
				shape.setAsBox(1.5f, 1.5f);

				BodyDef bd = new BodyDef();
				bd.type = BodyType.DYNAMIC;
				bd.position.set(0.0f, 17.0f);
				Body body = world.createBody(bd);
				body.createFixture(shape, 2.0f);

				RevoluteJointDef rjd = new RevoluteJointDef();
				rjd.initialize(prevBody, body, new Vec2(0.0f, 17.0f));
				world.createJoint(rjd);

				PrismaticJointDef pjd = new PrismaticJointDef();
				pjd.initialize(ground, body, new Vec2(0.0f, 17.0f), new Vec2(0.0f, 1.0f));

				pjd.maxMotorForce = 1000.0f;
				pjd.enableMotor = true;

				m_joint2 = (PrismaticJoint)world.createJoint(pjd);
			}

			// Create a payload
			{
				PolygonShape shape = new PolygonShape();
				shape.setAsBox(1.5f, 1.5f);

				BodyDef bd = new BodyDef();
				bd.type = BodyType.DYNAMIC;
				bd.position.set(0.0f, 23.0f);
				Body body = world.createBody(bd);
				body.createFixture(shape, 2.0f);
			}
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#step(org.jbox2d.testbed.framework.TestbedSettings)
	 */
	@Override
	public void step(TestbedSettings settings) {
		super.step(settings);
		
		addTextLine("Keys: (f) toggle friction, (m) toggle motor");
		float torque = m_joint1.getMotorTorque();
		Formatter f = new Formatter();
		addTextLine(f.format("Friction: %b, Motor Force = %5.0f, ", m_joint2.isMotorEnabled(), torque).toString());
		
		if(TestPanel.keys['f']){
			m_joint2.enableMotor(!m_joint2.isMotorEnabled());
			TestPanel.keys['f'] = false;
		}
		else if(TestPanel.keys['m']){
			m_joint1.enableMotor(!m_joint1.isMotorEnabled());
			TestPanel.keys['m'] = false;
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "SliderCrankTest";
	}
	
}

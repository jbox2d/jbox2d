/**
 * Created at 7:07:33 AM Jan 22, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class TOITest extends TestbedTest {
	
	PolygonShape m_shapeA = new PolygonShape();
	PolygonShape m_shapeB = new PolygonShape();
	RevoluteJoint m_joint;
	
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
			
			bd.position.set(0, 20);
			Body body = world.createBody(bd);
			body.createFixture(shape, 5.0f);
			
			float w = 100.0f;
			body.setAngularVelocity(w);
			body.setLinearVelocity(new Vec2(8.0f * w, 0.0f));
			
			rjd.initialize(ground, body, new Vec2(0.0f, 12.0f));
			rjd.motorSpeed = -1.0f * MathUtils.PI;
			rjd.maxMotorTorque = 10000.0f;
			rjd.enableMotor = false;
			rjd.lowerAngle = -0.25f * MathUtils.PI;
			rjd.upperAngle = 0.5f * MathUtils.PI;
			rjd.enableLimit = true;
			rjd.collideConnected = true;
			
			m_joint = (RevoluteJoint) world.createJoint(rjd);
		}
		
		Body small = null;
		{
			BodyDef bd = new BodyDef();
			bd.type = BodyType.DYNAMIC;
			bd.bullet = true;
			bd.position = new Vec2(5.66f, 7f);
			small = world.createBody(bd);
			
			CircleShape shape = new CircleShape();
			shape.m_radius = 0.3f;
			
			small.createFixture(shape, 10f);
			
			small.setLinearVelocity(new Vec2(0, 50));
		}
	}
	
	int times;
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#step(org.jbox2d.testbed.framework.TestbedSettings)
	 */
	@Override
	public void step(TestbedSettings settings) {
//		System.out.println("-------------------- "+times);
		// TODO Auto-generated method stub
		super.step(settings);
		times ++;
		
		if(times >= 20){
			//System.exit(0);
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "TOI Test";
	}
	
}

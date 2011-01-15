///**
// * Created at 4:25:03 AM Jan 15, 2011
// */
//package org.jbox2d.testbed.tests;
//
//import org.jbox2d.dynamics.joints.PrismaticJoint;
//import org.jbox2d.dynamics.joints.RevoluteJoint;
//import org.jbox2d.testbed.framework.TestbedSettings;
//import org.jbox2d.testbed.framework.TestbedTest;
//
///**
// * @author Daniel Murphy
// */
//public class Gears extends TestbedTest {
//	
//	RevoluteJoint m_joint1;
//	RevoluteJoint m_joint2;
//	PrismaticJoint m_joint3;
//	GearJoint m_joint4;
//	GearJoint m_joint5;
//	
//	/**
//	 * @see org.jbox2d.testbed.framework.TestbedTest#initTest()
//	 */
//	@Override
//	public void initTest() {
//		Body ground = null;
//		{
//			BodyDef bd = new BodyDef();
//			ground = world.createBody(bd);
//
//			PolygonShape shape = new PolygonShape();
//			shape.setAsEdge(new Vec2(50.0f, 0.0f), new Vec2(-50.0f, 0.0f));
//			ground.createFixture(shape, 0.0f);
//		}
//
//		{
//			CircleShape circle1;
//			circle1.m_radius = 1.0f;
//
//			CircleShape circle2;
//			circle2.m_radius = 2.0f;
//			
//			PolygonShape box;
//			box.setAsBox(0.5f, 5.0f);
//
//			BodyDef bd1;
//			bd1.type = BodyType.DYNAMIC;
//			bd1.position.set(-3.0f, 12.0f);
//			Body body1 = world.createBody(bd1);
//			body1.createFixture(&circle1, 5.0f);
//
//			RevoluteJointDef jd1;
//			jd1.bodyA = ground;
//			jd1.bodyB = body1;
//			jd1.localAnchorA = ground.getLocalPoint(bd1.position);
//			jd1.localAnchorB = body1.getLocalPoint(bd1.position);
//			jd1.referenceAngle = body1.getAngle() - ground.getAngle();
//			m_joint1 = (RevoluteJoint)world.createJoint(&jd1);
//
//			BodyDef bd2;
//			bd2.type = BodyType.DYNAMIC;
//			bd2.position.set(0.0f, 12.0f);
//			Body body2 = world.createBody(bd2);
//			body2.createFixture(&circle2, 5.0f);
//
//			RevoluteJointDef jd2;
//			jd2.initialize(ground, body2, bd2.position);
//			m_joint2 = (RevoluteJoint)world.createJoint(&jd2);
//
//			BodyDef bd3;
//			bd3.type = BodyType.DYNAMIC;
//			bd3.position.set(2.5f, 12.0f);
//			Body body3 = world.createBody(bd3);
//			body3.createFixture(&box, 5.0f);
//
//			PrismaticJointDef jd3;
//			jd3.initialize(ground, body3, bd3.position, new Vec2(0.0f, 1.0f));
//			jd3.lowerTranslation = -5.0f;
//			jd3.upperTranslation = 5.0f;
//			jd3.enableLimit = true;
//
//			m_joint3 = (PrismaticJoint)world.createJoint(&jd3);
//
//			GearJointDef jd4;
//			jd4.bodyA = body1;
//			jd4.bodyB = body2;
//			jd4.joint1 = m_joint1;
//			jd4.joint2 = m_joint2;
//			jd4.ratio = circle2.m_radius / circle1.m_radius;
//			m_joint4 = (GearJoint)world.createJoint(&jd4);
//
//			GearJointDef jd5;
//			jd5.bodyA = body2;
//			jd5.bodyB = body3;
//			jd5.joint1 = m_joint2;
//			jd5.joint2 = m_joint3;
//			jd5.ratio = -1.0f / circle2.m_radius;
//			m_joint5 = (GearJoint)world.createJoint(&jd5);
//		}
//	}
//	
//	/**
//	 * @see org.jbox2d.testbed.framework.TestbedTest#step(org.jbox2d.testbed.framework.TestbedSettings)
//	 */
//	@Override
//	public void step(TestbedSettings settings) {
//		super.step(settings);
//		
//		float ratio, value;
//		
//		ratio = m_joint4.getRatio();
//		value = m_joint1.getJointAngle() + ratio * m_joint2.getJointAngle();
//		debugDraw.drawString(5, m_textLine, "theta1 + %4.2f * theta2 = %4.2f", (float) ratio, (float) value);
//		m_textLine += 15;
//
//		ratio = m_joint5.getRatio();
//		value = m_joint2.getJointAngle() + ratio * m_joint3.getJointTranslation();
//		debugDraw.drawString(5, m_textLine, "theta2 + %4.2f * delta = %4.2f", (float) ratio, (float) value);
//		m_textLine += 15;
//	}
//	
//	/**
//	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
//	 */
//	@Override
//	public String getTestName() {
//		// TODO Auto-generated method stub
//		return null;
//	}
//	
//}

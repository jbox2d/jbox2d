/**
 * Created at 2:10:11 PM Jan 23, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.joints.DistanceJointDef;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class Web extends TestbedTest {
	
	Body m_bodies[] = new Body[4];
	Joint m_joints[] = new Joint[8];
	
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
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(0.5f, 0.5f);
			
			BodyDef bd = new BodyDef();
			bd.type = BodyType.DYNAMIC;
			
			bd.position.set(-5.0f, 5.0f);
			m_bodies[0] = m_world.createBody(bd);
			m_bodies[0].createFixture(shape, 5.0f);
			
			bd.position.set(5.0f, 5.0f);
			m_bodies[1] = m_world.createBody(bd);
			m_bodies[1].createFixture(shape, 5.0f);
			
			bd.position.set(5.0f, 15.0f);
			m_bodies[2] = m_world.createBody(bd);
			m_bodies[2].createFixture(shape, 5.0f);
			
			bd.position.set(-5.0f, 15.0f);
			m_bodies[3] = m_world.createBody(bd);
			m_bodies[3].createFixture(shape, 5.0f);
			
			DistanceJointDef jd = new DistanceJointDef();
			Vec2 p1 = new Vec2();
			Vec2 p2 = new Vec2();
			Vec2 d = new Vec2();
			
			jd.frequencyHz = 4.0f;
			jd.dampingRatio = 0.5f;
			
			jd.bodyA = ground;
			jd.bodyB = m_bodies[0];
			jd.localAnchorA.set(-10.0f, 0.0f);
			jd.localAnchorB.set(-0.5f, -0.5f);
			p1 = jd.bodyA.getWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB.getWorldPoint(jd.localAnchorB);
			d = p2.sub(p1);
			jd.length = d.length();
			m_joints[0] = m_world.createJoint(jd);
			
			jd.bodyA = ground;
			jd.bodyB = m_bodies[1];
			jd.localAnchorA.set(10.0f, 0.0f);
			jd.localAnchorB.set(0.5f, -0.5f);
			p1 = jd.bodyA.getWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB.getWorldPoint(jd.localAnchorB);
			d = p2.sub(p1);
			jd.length = d.length();
			m_joints[1] = m_world.createJoint(jd);
			
			jd.bodyA = ground;
			jd.bodyB = m_bodies[2];
			jd.localAnchorA.set(10.0f, 20.0f);
			jd.localAnchorB.set(0.5f, 0.5f);
			p1 = jd.bodyA.getWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB.getWorldPoint(jd.localAnchorB);
			d = p2.sub(p1);
			jd.length = d.length();
			m_joints[2] = m_world.createJoint(jd);
			
			jd.bodyA = ground;
			jd.bodyB = m_bodies[3];
			jd.localAnchorA.set(-10.0f, 20.0f);
			jd.localAnchorB.set(-0.5f, 0.5f);
			p1 = jd.bodyA.getWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB.getWorldPoint(jd.localAnchorB);
			d = p2.sub(p1);
			jd.length = d.length();
			m_joints[3] = m_world.createJoint(jd);
			
			jd.bodyA = m_bodies[0];
			jd.bodyB = m_bodies[1];
			jd.localAnchorA.set(0.5f, 0.0f);
			jd.localAnchorB.set(-0.5f, 0.0f);;
			p1 = jd.bodyA.getWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB.getWorldPoint(jd.localAnchorB);
			d = p2.sub(p1);
			jd.length = d.length();
			m_joints[4] = m_world.createJoint(jd);
			
			jd.bodyA = m_bodies[1];
			jd.bodyB = m_bodies[2];
			jd.localAnchorA.set(0.0f, 0.5f);
			jd.localAnchorB.set(0.0f, -0.5f);
			p1 = jd.bodyA.getWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB.getWorldPoint(jd.localAnchorB);
			d = p2.sub(p1);
			jd.length = d.length();
			m_joints[5] = m_world.createJoint(jd);
			
			jd.bodyA = m_bodies[2];
			jd.bodyB = m_bodies[3];
			jd.localAnchorA.set(-0.5f, 0.0f);
			jd.localAnchorB.set(0.5f, 0.0f);
			p1 = jd.bodyA.getWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB.getWorldPoint(jd.localAnchorB);
			d = p2.sub(p1);
			jd.length = d.length();
			m_joints[6] = m_world.createJoint(jd);
			
			jd.bodyA = m_bodies[3];
			jd.bodyB = m_bodies[0];
			jd.localAnchorA.set(0.0f, -0.5f);
			jd.localAnchorB.set(0.0f, 0.5f);
			p1 = jd.bodyA.getWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB.getWorldPoint(jd.localAnchorB);
			d = p2.sub(p1);
			jd.length = d.length();
			m_joints[7] = m_world.createJoint(jd);
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#keyPressed(char, int)
	 */
	@Override
	public void keyPressed(char key, int argKeyCode) {
		switch (key) {
			case 'b' :
				for (int i = 0; i < 4; ++i) {
					if (m_bodies[i] != null) {
						m_world.destroyBody(m_bodies[i]);
						m_bodies[i] = null;
						break;
					}
				}
				break;
			
			case 'j' :
				for (int i = 0; i < 8; ++i) {
					if (m_joints[i] != null) {
						m_world.destroyJoint(m_joints[i]);
						m_joints[i] = null;
						break;
					}
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
		addTextLine("This demonstrates a soft distance joint.");
		addTextLine("Press: (b) to delete a body, (j) to delete a joint");
	}
	
	public void jointDestroyed(Joint joint) {
		for (int i = 0; i < 8; ++i) {
			if (m_joints[i] == joint) {
				m_joints[i] = null;
				break;
			}
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "Web";
	}
	
}

// Needs 2.0.1 update before finishing port
/*
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.PolygonDef;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;
*/
/**
 * Inspired by a contribution by roman_m
 * Dimensions scooped from APE (http://www.cove.org/ape/index.htm)
 */
/*
public class TheoJansen extends AbstractExample {
	Vec2 m_offset;
	Body m_chassis;
	Body m_wheel;
	RevoluteJoint m_motorJoint;
	boolean m_motorOn;
	float m_motorSpeed;
	
	public TheoJansen(TestbedMain p) {
        super(p);
    }
    
    public String getName() {
    	return "Theo Jansen Walker";
    }
    
    void createLeg(float s, Vec2 wheelAnchor) {
		Vec2 p1 = new Vec2(5.4f * s, -6.1f);
		Vec2 p2 = new Vec2(7.2f * s, -1.2f);
		Vec2 p3 = new Vec2(4.3f * s, -1.9f);
		Vec2 p4 = new Vec2(3.1f * s, 0.8f);
		Vec2 p5 = new Vec2(6.0f * s, 1.5f);
		Vec2 p6 = new Vec2(2.5f * s, 3.7f);

		PolygonDef sd1 = new PolygonDef();
		PolygonDef sd2 = new PolygonDef();
		
		sd1.filter.groupIndex = -1;
		sd2.filter.groupIndex = -1;
		sd1.density = 1.0f;
		sd2.density = 1.0f;

		if (s > 0.0f)
		{
			sd1.vertices[0] = p1;
			sd1.vertices[1] = p2;
			sd1.vertices[2] = p3;

			sd2.vertices[0] = Vec2_zero;
			sd2.vertices[1] = p5 - p4;
			sd2.vertices[2] = p6 - p4;
		}
		else
		{
			sd1.vertices[0] = p1;
			sd1.vertices[1] = p3;
			sd1.vertices[2] = p2;

			sd2.vertices[0] = Vec2_zero;
			sd2.vertices[1] = p6 - p4;
			sd2.vertices[2] = p5 - p4;
		}

		b2BodyDef bd1, bd2;
		bd1.position = m_offset;
		bd2.position = p4 + m_offset;

		bd1.angularDamping = 10.0f;
		bd2.angularDamping = 10.0f;

		b2Body* body1 = m_world.createBody(&bd1);
		b2Body* body2 = m_world.createBody(&bd2);

		body1.createShape(&sd1);
		body2.createShape(&sd2);

		body1.setMassFromShapes();
		body2.setMassFromShapes();

		b2DistanceJointDef djd;

		djd.Initialize(body1, body2, p2 + m_offset, p5 + m_offset);
		m_world.createJoint(&djd);

		djd.Initialize(body1, body2, p3 + m_offset, p4 + m_offset);
		m_world.createJoint(&djd);

		djd.Initialize(body1, m_wheel, p3 + m_offset, wheelAnchor + m_offset);
		m_world.createJoint(&djd);

		djd.Initialize(body2, m_wheel, p6 + m_offset, wheelAnchor + m_offset);
		m_world.createJoint(&djd);

		b2RevoluteJointDef rjd;

		rjd.Initialize(body2, m_chassis, p4 + m_offset);
		m_world.createJoint(&rjd);
	}

	void TheoJansen(){
		m_offset.set(0.0f, 8.0f);
		m_motorSpeed = 2.0f;
		m_motorOn = true;
		Vec2 pivot = new Vec2(0.0f, 0.8f);

		{
			b2PolygonDef sd;
			sd.SetAsBox(50.0f, 10.0f);

			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);
			b2Body* ground = m_world.createBody(&bd);
			ground.createShape(&sd);

			sd.SetAsBox(0.5f, 5.0f, Vec2(-50.0f, 15.0f), 0.0f);
			ground.createShape(&sd);

			sd.SetAsBox(0.5f, 5.0f, Vec2(50.0f, 15.0f), 0.0f);
			ground.createShape(&sd);
		}

		for (int32 i = 0; i < 40; ++i)
		{
			b2CircleDef sd;
			sd.density = 1.0f;
			sd.radius = 0.25f;

			b2BodyDef bd;
			bd.position.Set(-40.0f + 2.0f * i, 0.5f);

			b2Body* body = m_world.createBody(&bd);
			body.createShape(&sd);
			body.setMassFromShapes();
		}

		{
			b2PolygonDef sd;
			sd.density = 1.0f;
			sd.SetAsBox(2.5f, 1.0f);
			sd.filter.groupIndex = -1;
			b2BodyDef bd;
			bd.position = pivot + m_offset;
			m_chassis = m_world.createBody(&bd);
			m_chassis.createShape(&sd);
			m_chassis.setMassFromShapes();
		}

		{
			b2CircleDef sd;
			sd.density = 1.0f;
			sd.radius = 1.6f;
			sd.filter.groupIndex = -1;
			b2BodyDef bd;
			bd.position = pivot + m_offset;
			m_wheel = m_world.createBody(&bd);
			m_wheel.createShape(&sd);
			m_wheel.setMassFromShapes();
		}

		{
			b2RevoluteJointDef jd;
			jd.Initialize(m_wheel, m_chassis, pivot + m_offset);
			jd.collideConnected = false;
			jd.motorSpeed = m_motorSpeed;
			jd.maxMotorTorque = 400.0f;
			jd.enableMotor = m_motorOn;
			m_motorJoint = (b2RevoluteJoint*)m_world.createJoint(&jd);
		}

		Vec2 wheelAnchor;
		
		wheelAnchor = pivot + Vec2(0.0f, -0.8f);

		CreateLeg(-1.0f, wheelAnchor);
		CreateLeg(1.0f, wheelAnchor);

		m_wheel.setXForm(m_wheel.getPosition(), 120.0f * b2_pi / 180.0f);
		CreateLeg(-1.0f, wheelAnchor);
		CreateLeg(1.0f, wheelAnchor);

		m_wheel.setXForm(m_wheel.getPosition(), -120.0f * b2_pi / 180.0f);
		CreateLeg(-1.0f, wheelAnchor);
		CreateLeg(1.0f, wheelAnchor);
	}

	void Step(Settings* settings)
	{
		DrawString(5, m_textLine, "Keys: left = a, brake = s, right = d, toggle motor = m");
		m_textLine += 15;

		Test::Step(settings);
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 'a':
			m_chassis->WakeUp();
			m_motorJoint->SetMotorSpeed(-m_motorSpeed);
			break;

		case 's':
			m_chassis->WakeUp();
			m_motorJoint->SetMotorSpeed(0.0f);
			break;

		case 'd':
			m_chassis->WakeUp();
			m_motorJoint->SetMotorSpeed(m_motorSpeed);
			break;

		case 'm':
			m_chassis->WakeUp();
			m_motorJoint->EnableMotor(!m_motorJoint->IsMotorEnabled());
			break;
		}
	}
}*/

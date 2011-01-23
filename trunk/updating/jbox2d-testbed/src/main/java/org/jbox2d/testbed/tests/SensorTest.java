/**
 * Created at 1:25:51 PM Jan 23, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class SensorTest extends TestbedTest {
	
	class BoolWrapper{
		boolean tf;
	}
	int e_count = 7;
	Fixture m_sensor;
	Body m_bodies[] = new Body[e_count];
	BoolWrapper m_touching[] = new BoolWrapper[e_count];
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#initTest()
	 */
	@Override
	public void initTest() {
		
		for(int i=0; i<m_touching.length; i++){
			m_touching[i] = new BoolWrapper();
		}
		
		{
			BodyDef bd = new BodyDef();
			Body ground = m_world.createBody(bd);
			
			{
				PolygonShape shape = new PolygonShape();
				shape.setAsEdge(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
				ground.createFixture(shape, 0.0f);
			}
			
			{
				CircleShape shape = new CircleShape();
				shape.m_radius = 5.0f;
				shape.m_p.set(0.0f, 10.0f);
				
				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.isSensor = true;
				m_sensor = ground.createFixture(fd);
			}
		}
		
		{
			CircleShape shape = new CircleShape();
			shape.m_radius = 1.0f;
			
			for (int i = 0; i < e_count; ++i) {
				BodyDef bd = new BodyDef();
				bd.type = BodyType.DYNAMIC;
				bd.position.set(-10.0f + 3.0f * i, 20.0f);
				bd.userData = m_touching[i];
				
				m_touching[i].tf = false;
				m_bodies[i] = m_world.createBody(bd);
				
				m_bodies[i].createFixture(shape, 1.0f);
			}
		}
	}
	
	// Implement contact listener.
	public void beginContact(Contact contact) {
		Fixture fixtureA = contact.getFixtureA();
		Fixture fixtureB = contact.getFixtureB();
		
		if (fixtureA == m_sensor) {
			Object userData = fixtureB.getBody().getUserData();
			if (userData != null) {
				((BoolWrapper)userData).tf = true;
			}
		}
		
		if (fixtureB == m_sensor) {
			Object userData = fixtureA.getBody().getUserData();
			if (userData != null) {
				((BoolWrapper)userData).tf = true;
			}
		}
	}
	
	// Implement contact listener.
	public void endContact(Contact contact) {
		Fixture fixtureA = contact.getFixtureA();
		Fixture fixtureB = contact.getFixtureB();
		
		if (fixtureA == m_sensor) {
			Object userData = fixtureB.getBody().getUserData();
			if (userData != null) {
				((BoolWrapper)userData).tf = false;
			}
		}
		
		if (fixtureB == m_sensor) {
			Object userData = fixtureA.getBody().getUserData();
			if (userData != null) {
				((BoolWrapper)userData).tf = false;
			}
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#step(org.jbox2d.testbed.framework.TestbedSettings)
	 */
	@Override
	public void step(TestbedSettings settings) {
		// TODO Auto-generated method stub
		super.step(settings);
		
		// Traverse the contact results. Apply a force on shapes
		// that overlap the sensor.
		for (int i = 0; i < e_count; ++i) {
			if (m_touching[i].tf == false) {
				continue;
			}
			
			Body body = m_bodies[i];
			Body ground = m_sensor.getBody();
			
			CircleShape circle = (CircleShape) m_sensor.getShape();
			Vec2 center = ground.getWorldPoint(circle.m_p);
			
			Vec2 position = body.getPosition();
			
			Vec2 d = center.sub(position);
			if (d.lengthSquared() < Settings.EPSILON * Settings.EPSILON) {
				continue;
			}
			
			d.normalize();
			Vec2 F = d.mulLocal(100f);
			body.applyForce(F, position);
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "Sensor Test";
	}
	
}

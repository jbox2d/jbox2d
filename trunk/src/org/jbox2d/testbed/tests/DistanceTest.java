package org.jbox2d.testbed.tests;

import org.jbox2d.collision.CircleDef;
import org.jbox2d.collision.Distance;
import org.jbox2d.collision.PolygonDef;
import org.jbox2d.collision.Shape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestSettings;
import org.jbox2d.testbed.TestbedMain;

public class DistanceTest extends AbstractExample {
	Body m_body1;
	Body m_body2;
	Shape m_shape1;
	Shape m_shape2;
	
	private boolean firstTime = true;
	
	public DistanceTest(TestbedMain _parent) {
		super(_parent);
	}
	
	public void create() {
		if (firstTime) {
			setCamera(0.0f,10.0f,20.0f);
			firstTime = false;
		}

		{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(1.0f, 1.0f);
			sd.density = 0.0f;

			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, 10.0f);
			m_body1 = m_world.createStaticBody(bd);
			m_shape1 = m_body1.createShape(sd);
		}
		
		{
			/*PolygonDef sd = new PolygonDef();
			sd.vertices.add(new Vec2(-1.0f, 0.0f));
			sd.vertices.add(new Vec2(1.0f, 0.0f));
			sd.vertices.add(new Vec2(0.0f, 15.0f));*/
			CircleDef sd = new CircleDef();
			sd.radius = 2.0f;
			sd.density = 1.0f;

			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, 10.0f);
			m_body2 = m_world.createDynamicBody(bd);
			m_shape2 = m_body2.createShape(sd);
			m_body2.setMassFromShapes();
		}

		m_world.m_gravity.set(0.0f, 0.0f);
	}

	public void step() {
		settings.pause = true;
		settings.enablePositionCorrection = false;
		super.step();
		settings.enablePositionCorrection = true;
		settings.pause = false;

		Vec2 x1 = new Vec2();
		Vec2 x2 = new Vec2();
		float distance = Distance.distance(x1, x2, m_shape1, m_body1.getXForm(), m_shape2, m_body2.getXForm());

		m_debugDraw.drawString(5, m_textLine, "distance = "+distance, white);
		m_textLine += 15;

		m_debugDraw.drawString(5, m_textLine, "iterations = "+Distance.g_GJK_Iterations, white);
		m_textLine += 15;

		m_debugDraw.drawPoint(x1, 2.0f, white);
		m_debugDraw.drawPoint(x2, 2.0f, white);
		m_debugDraw.drawSegment(x1,x2,white);		
	}

	public void keyPressed(int key) {
		Vec2 p = m_body2.getPosition();
		float a = m_body2.getAngle();

		switch (key) {
		case 'a':
			p.x -= 0.1f;
			break;

		case 'd':
			p.x += 0.1f;
			break;

		case 's':
			p.y -= 0.1f;
			break;

		case 'w':
			p.y += 0.1f;
			break;

		case 'q':
			a += 0.1f * (float)Math.PI;
			break;

		case 'e':
			a -= 0.1f * (float)Math.PI;
			break;
		}

		m_body2.setXForm(p, a);
	}

	public String getName() {
		return "Distance Test";
	}

}

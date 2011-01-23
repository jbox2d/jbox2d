/**
 * Created at 4:56:29 AM Jan 14, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class VerticalStack extends TestbedTest {
	
	public static final int e_columnCount = 5;
	public static final int e_rowCount = 16;
	
	Body m_bullet;
	Body m_bodies[] = new Body[e_rowCount * e_columnCount];
	int m_indices[] = new int[e_rowCount * e_columnCount];
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#initTest()
	 */
	@Override
	public void initTest() {
		{
			BodyDef bd = new BodyDef();
			Body ground = m_world.createBody(bd);

			PolygonShape shape = new PolygonShape();
			shape.setAsEdge(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
			ground.createFixture(shape, 0.0f);

			shape.setAsEdge(new Vec2(20.0f, 0.0f), new Vec2(20.0f, 20.0f));
			ground.createFixture(shape, 0.0f);
		}

		float xs[] = new float[]{0.0f, -10.0f, -5.0f, 5.0f, 10.0f};

		for (int j = 0; j < e_columnCount; ++j)
		{
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(0.5f, 0.5f);

			FixtureDef fd = new FixtureDef();
			fd.shape = shape;
			fd.density = 1.0f;
			fd.friction = 0.3f;

			for (int i = 0; i < e_rowCount; ++i)
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType.DYNAMIC;

				int n = j * e_rowCount + i;
				assert(n < e_rowCount * e_columnCount);
				m_indices[n] = n;
				bd.userData = m_indices[n];

				float x = 0.0f;
				//float x = RandomFloat(-0.02f, 0.02f);
				//float x = i % 2 == 0 ? -0.025f : 0.025f;
				bd.position.set(xs[j] + x, 0.752f + 1.54f * i);
				Body body = m_world.createBody(bd);

				m_bodies[n] = body;

				body.createFixture(fd);
			}
		}

		m_bullet = null;
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#keyPressed(char, int)
	 */
	@Override
	public void keyPressed(char argKeyChar, int argKeyCode) {
		switch (argKeyChar)
		{
		case ',':
			if (m_bullet != null)
			{
				m_world.destroyBody(m_bullet);
				m_bullet = null;
			}

			{
				CircleShape shape = new CircleShape();
				shape.m_radius = 0.25f;

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.density = 20.0f;
				fd.restitution = 0.05f;

				BodyDef bd = new BodyDef();
				bd.type = BodyType.DYNAMIC;
				bd.bullet = true;
				bd.position.set(-31.0f, 5.0f);

				m_bullet = m_world.createBody(bd);
				m_bullet.createFixture(fd);

				m_bullet.setLinearVelocity(new Vec2(400.0f, 0.0f));
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
		addTextLine("Press ',' to launch bullet.");
	}

	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "Vertical Stack";
	}

}

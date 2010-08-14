/**
 * Created at 5:36:06 PM Jul 17, 2010
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class PyramidTest extends TestbedTest {
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#initTest()
	 */
	@Override
	public void initTest() {
		setTitle("Pyramid Test");
		int count = 20;
		{
			BodyDef bd = new BodyDef();
			Body ground = world.createBody(bd);

			PolygonShape shape = new PolygonShape();
			shape.setAsEdge(new Vec2(-40.0f, -10.0f), new Vec2(40.0f, -10.0f));
			ground.createFixture(shape, 0.0f);
			
//			CircleShape cs = new CircleShape();
//			cs.m_radius = 8.0f;
//			cs.m_p.set(-0.3f,-8.0f);
//			Fixture f = ground.createFixture(cs, 0.0f);
//			f.setFriction(0.0f);
			
		}

		{
			float a = .5f;
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(a, a);
			
			Vec2 x = new Vec2(-7.0f, 0.75f);
			Vec2 y = new Vec2();
			Vec2 deltaX = new Vec2(0.5625f, 1.25f);
			Vec2 deltaY = new Vec2(1.125f, 0.0f);

			for (int i = 0; i < count; ++i){
				y.set(x);

				for (int j = i; j < count; ++j){
					BodyDef bd = new BodyDef();
					bd.type = BodyType.DYNAMIC;
					bd.position.set(y);
					Body body = world.createBody(bd);
					body.createFixture(shape, 5.0f);
					y.addLocal(deltaY);
				}

				x.addLocal(deltaX);
			}
		}
	}

	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "Pyramid";
	}

}

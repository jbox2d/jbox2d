/**
 * 
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.Segment;
import org.jbox2d.collision.shapes.CircleDef;
import org.jbox2d.collision.shapes.PolygonDef;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.RaycastResult;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;

public class RaycastTest extends AbstractExample {

	Body laserBody;
	
	public RaycastTest(TestbedMain _parent) {
		super(_parent);
	}
	
	public void create() {
		Body ground = null;
		{
			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, -10.0f);
			ground = m_world.createBody(bd);

			PolygonDef sd = new PolygonDef();
			sd.setAsBox(50.0f, 10.0f);
			ground.createShape(sd);
		}

		{
			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, 1.0f);
			laserBody = m_world.createBody(bd);

			PolygonDef sd = new PolygonDef();
			sd.setAsBox(5.0f, 1.0f);
			sd.density = 4.0f;
			laserBody.createShape(sd);
			laserBody.setMassFromShapes();

			Body body;
			//Create a few shapes
			bd.position.set(-5.0f, 10.0f);
			body = m_world.createBody(bd);

			CircleDef cd = new CircleDef();
			cd.radius = 3;
			body.createShape(cd);

			bd.position.set(5.0f, 10.0f);
			body = m_world.createBody(bd);

			body.createShape(cd);
		}
	}
	
	@Override
	public void step() {
		super.step();
//		if (true) return;
		float segmentLength = 30.0f;

		Segment segment = new Segment();
		Vec2 laserStart = new Vec2(5.0f-.1f,0.0f);
		Vec2 laserDir = new Vec2(segmentLength,0.0f);
		segment.p1.set(laserBody.getWorldLocation(laserStart));
		segment.p2.set(laserBody.getWorldDirection(laserDir));
		segment.p2.addLocal(segment.p1);

		for(int rebounds=0;rebounds<10;rebounds++){

			RaycastResult result = new RaycastResult();
			Shape shape = m_world.raycastOne(segment,result,false,null);

			float lambda = result.lambda;
			Vec2 normal = result.normal;
			
			Color3f laserColor = new Color3f(255,0,0);

			if(shape != null)
			{
				Vec2 endVec = segment.p1.mul(1-lambda).add(segment.p2.mul(lambda));
				m_debugDraw.drawSegment(segment.p1,endVec,laserColor);
			}
			else
			{
				m_debugDraw.drawSegment(segment.p1,segment.p2,laserColor);
				break;
			}
			//Bounce
			segmentLength *= (1-lambda);
			if(segmentLength<=Settings.EPSILON) break;
			laserStart = segment.p1.mul(1-lambda).add(segment.p2.mul(lambda));
			laserDir = segment.p2.sub(segment.p1);
			laserDir.normalize();
			laserDir = laserDir.add(normal.mul(-2 * Vec2.dot(laserDir,normal)));
			segment.p1.set(laserStart.sub(laserDir.mul(0.1f)));
			segment.p2.set(laserStart.add(laserDir.mul(segmentLength)));
			
		}
	}
	
	public String getName() {
		return "Raycast Test";
	}
}

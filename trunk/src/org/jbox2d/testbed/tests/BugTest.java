package org.jbox2d.testbed.tests;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.shapes.PolygonDef;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;
import org.jbox2d.util.nonconvex.Polygon;

public class BugTest extends AbstractExample {
	private boolean firstTime = true;
	
	@Override
	public void createWorld() {
		m_worldAABB = new AABB();
		m_worldAABB.lowerBound.set(-10000.0f, -10000.0f);
		m_worldAABB.upperBound.set(10000.0f, 10000.0f);
		Vec2 gravity = new Vec2(0.0f, -10.0f);
		boolean doSleep = true;
		m_world = new World(m_worldAABB, gravity, doSleep);
	}
	
    public BugTest(TestbedMain t) {
        super(t);
    }
    
    public String getName() {
    	return "Bug Test";
    }
    
    public String getExampleInstructions() {
    	return "";
    }

    @Override
    public void create() {
    	if (firstTime) {
			setCamera(2f, 12f, 10f);
			firstTime = false;
		}
    	
    	Body ground = null;
		{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(500.0f, 0.2f);

			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, 0.0f);
			ground = m_world.createBody(bd);
			ground.createShape(sd);
		}
    	
    	//new PistonBenchmark().create(m_world);
    	Vec2[] vecs = {
    			new Vec2(96,224),
    			new Vec2(96,288),
    			new Vec2(32,288),
    			new Vec2(32,320),
    			new Vec2(96,320),
    			new Vec2(96,384),
    			new Vec2(128,384),
    			new Vec2(128,320),
    			new Vec2(160,320),
    			new Vec2(160,384),
    			new Vec2(192,384),
    			new Vec2(192,320),
    			new Vec2(256,320),
    			new Vec2(256,288),
    			new Vec2(192,288),
    			new Vec2(192,224),
    			new Vec2(160,224),
    			new Vec2(160,288),
    			new Vec2(128,288),
    			new Vec2(128,224)
    	};
    	
    	for (Vec2 v:vecs) {
    		v.x *= 1.00f;
    		v.y *= 1.00f;
    	}
    	
//    	float xv[] = {160.000000f,160.000000f,128.000000f,128.000000f,};
//    	float yv[] = {224.000000f,288.000000f,288.000000f,224.000000f,};
    	
    	float xv[] = {1888,1888,640,608,576,480,448,384,352,0,  0,  900};
    	float yv[] = {448, 416, 416,384,416,416,384,384,416,416,448, 448};
    	
    	//Polygon myPoly = new Polygon(vecs);
    	Polygon myPoly = new Polygon(xv,yv);
    	PolygonDef pd = new PolygonDef();
    	pd.density = 1.0f;
    	BodyDef bd = new BodyDef();
    	bd.position.set(0.0f,6.0f);
    	Body body = m_world.createBody(bd);
    	Polygon.decomposeConvexAndAddTo(myPoly, body, pd);
    	body.setMassFromShapes();
    	/*
    	PolygonDef pd = new PolygonDef();
    	pd.setAsBox(5.0f, 5.0f);
    	pd.isSensor = true;
    	BodyDef bd = new BodyDef();
    	bd.position.set(0.0f,0.0f);
    	Body bod = m_world.createBody(bd);
    	bod.createShape(pd);
    	
    	CircleDef cd = new CircleDef();
    	cd.radius = 0.5f;
    	cd.density = 1.0f;
    	bd = new BodyDef();
    	bd.position.set(0.0f,15.0f);
    	Body b = m_world.createBody(bd);
    	b.createShape(cd);
    	b.setMassFromShapes();
    	b.setLinearVelocity(new Vec2(0.0f,0.0f));
    }
    
	public void postStep() {		
		for (int i=0; i<this.m_pointCount; ++i) {
			ExampleContactPoint cp = m_points[i];
			if (cp.state == 1) return;
			if (cp.state == 0) System.out.println("Add");
			//else if (cp.state == 1) System.out.println("Persist");
			else if (cp.state == 2) System.out.println("Remove");
			System.out.println(cp.shape1 + " " +cp.shape2);
			System.out.println(cp.position);
		}*/
	}
    int count = 0;
    public void postStep() {
    	//System.out.println(++count);
    }
}

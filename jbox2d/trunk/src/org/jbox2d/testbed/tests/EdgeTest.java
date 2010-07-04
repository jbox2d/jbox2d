package org.jbox2d.testbed.tests;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.shapes.CircleDef;
import org.jbox2d.collision.shapes.EdgeChainDef;
import org.jbox2d.collision.shapes.PolygonDef;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;

public class EdgeTest extends AbstractExample {
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
	
    public EdgeTest(TestbedMain t) {
        super(t);
    }
    
    public String getName() {
    	return "Edge Shape Test";
    }
    
    public String getExampleInstructions() {
    	return "";
    }

    @Override
    public void create() {
    	if (firstTime) {
			setCamera(0f, 70f, 3f);
			firstTime = false;
		}
    	
    	EdgeChainDef myEdges = new EdgeChainDef();
    	final float delta = 1.0f;
    	xOffset = 100.0f * (float)Math.random();
    	for (float x = 100.0f; x > -100.0f; x -= delta) {
    		myEdges.addVertex(getHeightField(x));
    	}
    	myEdges.setIsLoop(false);
    	myEdges.friction = 2.0f;
    	
    	BodyDef bd = new BodyDef();
    	bd.position.set(0.0f,0.0f);
    	Body body = m_world.createBody(bd);
    	body.createShape(myEdges);
    	
    	// Make some bodies
    	int nBodies = 200;
    	for (int i=0; i<nBodies; ++i) {
    		float x = -80.0f + 20.0f*(float)(Math.random()-0.5f);
    		float y = 110.0f + 80.0f*(float)(Math.random());
    		float sz = 0.2f + 0.3f*(float)Math.random();
//    		if (i%2 == 0) { 
    			makeCircle(x,y,sz);
//    		} else { 
//    			makeBox(x,y,sz);
//    		}
    	}
    	
	}
    
    private final float variation = 40.0f;
    private final float xScale = 30.0f;
    float xOffset;
    private Vec2 getHeightField(float xin) {
    	float xout = xin;
    	float yout = .01f*xin*xin + parent.noise(xin/xScale + xOffset)*variation;
    	return new Vec2(xout, yout);
    }
    
    public void makeCircle(float x, float y, float r) {
    	BodyDef bd = new BodyDef();
    	bd.position = new Vec2(x,y);
    	Body b = m_world.createBody(bd);
    	CircleDef cd = new CircleDef();
    	cd.radius = r;
    	cd.density = 1.0f;
    	cd.friction = 0.01f;
    	b.createShape(cd);
    	b.setMassFromShapes();
    }
    
    public void makeBox(float x, float y, float size) {
    	BodyDef bd = new BodyDef();
    	bd.position = new Vec2(x,y);
    	Body b = m_world.createBody(bd);
    	PolygonDef pd = new PolygonDef();
    	pd.setAsBox(size, size,new Vec2(), 2*3.1415f*(float)Math.random());
    	pd.density = 1.0f;
    	pd.friction = 0.01f;
    	b.createShape(pd);
    	b.setMassFromShapes();
    }
    
    public void postStep() {
    	//System.out.println(++count);
    }
}
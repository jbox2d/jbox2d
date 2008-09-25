package org.jbox2d.testbed.tests;

import org.jbox2d.collision.CircleDef;
import org.jbox2d.collision.PolygonDef;
import org.jbox2d.collision.Shape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.joints.PrismaticJoint;
import org.jbox2d.dynamics.joints.PrismaticJointDef;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.ExampleContactPoint;
import org.jbox2d.testbed.TestbedMain;
import org.jbox2d.testbed.timingTests.PistonBenchmark;

public class BugTest extends AbstractExample {
	private boolean firstTime = true;
	
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
    	
    	new PistonBenchmark().create(m_world);
    	
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
    	System.out.println(++count);
    }
}

/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.box2d.org
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package org.jbox2d.testbed.tests;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.CircleDef;
import org.jbox2d.collision.PolygonDef;
import org.jbox2d.collision.Shape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.ExampleContactPoint;
import org.jbox2d.testbed.TestbedMain;
import org.jbox2d.util.blob.*;

public class SensorTest extends AbstractExample {
	private boolean firstTime;
	private float springConstant = 15.5f;
	private float springFriction = 0.40f;
	private float springDistance = 1.0f;
	float size = 0.2f;
	float density = 0.25f;
	float sensorSize = 0.5f;

	int nParts = 700;
	
	public SensorTest(TestbedMain _parent) {
		super(_parent);
		firstTime = true;
	}
	
	@Override
	public void create() {

		if (firstTime) {
			setCamera(0.0f,10.0f,20.0f);
			firstTime = false;
		}
		
    	Body ground = null;
		{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(50.0f, 0.2f);

			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, 0.0f);
			ground = m_world.createBody(bd);
			ground.createShape(sd);
			
			// Walls
			
            sd.setAsBox(1.0f,58.0f);
            bd = new BodyDef();
            bd.position = new Vec2(7.0f,64.0f);
            bd.angle = -0.08f;
            Body rightWall = m_world.createBody(bd);
            rightWall.createShape(sd);
            bd.position = new Vec2(-7.0f,64.0f);
            bd.angle = 0.08f;
            Body leftWall = m_world.createBody(bd);
            leftWall.createShape(sd);
            
            /*
            // Corners 
            bd = new BodyDef();
            sd.setAsBox(20.0f,3.0f);
            bd.angle = (float)(-Math.PI/4.0);
            bd.position = new Vec2(-40f,0.0f);
            Body myBod = m_world.createBody(bd);
            myBod.createShape(sd);
            bd.angle = (float)(Math.PI/4.0);
            bd.position = new Vec2(40f,0.0f);
            myBod = m_world.createBody(bd);
            myBod.createShape(sd);*/
		}
		
		
		for (int i=0; i<nParts; ++i) {
			float x = parent.random(-4f,4f);
			float y = parent.random(30.0f,120.0f);
			createParticle(new Vec2(x,y));
		}
		
		
	}
	
	public void postStep() {
		
		for (int i=0; i<this.m_pointCount; ++i) {
			ExampleContactPoint cp = m_points[i];
			Shape s1 = cp.shape1;
			Shape s2 = cp.shape2;
			if (s1.getUserData() == null || s2.getUserData() == null) {
				continue;
			}
			Body b1 = s1.getBody();
			Body b2 = s2.getBody();
			if (b1.getPosition().sub(b2.getPosition()).lengthSquared() > 2*sensorSize) continue;
			addSpringForce(b1,b2,springConstant,springFriction,springDistance);
		}
	}
	
	public void createParticle(Vec2 pos) {
		
		BodyDef bd = new BodyDef();
		bd.position = pos.clone();
		//bd.fixedRotation = true;
		Body body = m_world.createBody(bd);
		CircleDef cd = new CircleDef();
		cd.density = density;
		cd.radius = size;
		cd.restitution = 0.0f;
		body.createShape(cd);
		body.setMassFromShapes();
		cd.isSensor = true;
		cd.radius = sensorSize + parent.random(-.1f,0.1f);
		cd.userData = new Integer(1);
		body.createShape(cd);
	}
	
	static private final Vec2 ZEROVEC = new Vec2(0.0f,0.0f);
	public void addSpringForce(Body bA, Body bB, float k, float friction, float desiredDist) {
		addSpringForce(bA,ZEROVEC,bB,ZEROVEC,k,friction,desiredDist);
	}
	
	public void addSpringForce(Body bA, Vec2 localA, Body bB, Vec2 localB, float k, float friction, float desiredDist) {
        Vec2 pA = bA.getWorldPoint(localA);
        Vec2 pB = bB.getWorldPoint(localB);
        Vec2 diff = pB.sub(pA);
        //Find velocities of attach points
        Vec2 vA = bA.m_linearVelocity.sub(Vec2.cross(bA.getWorldVector(localA), bA.m_angularVelocity));
        Vec2 vB = bB.m_linearVelocity.sub(Vec2.cross(bB.getWorldVector(localB), bB.m_angularVelocity));
        Vec2 vdiff = vB.sub(vA);
        float dx = diff.normalize(); //normalizes diff and puts length into dx
        float vrel = vdiff.x*diff.x + vdiff.y*diff.y;
        float forceMag = -k*(dx-desiredDist) - friction*vrel;
        //System.out.println(dx+" "+desiredDist);
        diff.mulLocal(forceMag); // diff *= forceMag
        bB.applyForce(diff, bA.getWorldPoint(localA));
        bA.applyForce(diff.mulLocal(-1f), bB.getWorldPoint(localB));
        bA.wakeUp();
        bB.wakeUp();
    }
	
	
	@Override
	public String getName() {
		return "Sensor Test";
	}

}
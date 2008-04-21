/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.gphysics.com
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
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;

import processing.core.PApplet;

public class SpringRestitutionTest extends AbstractExample {
	//Used to calculate restitution vs. floor
	
	//Matches bodyC
	public Body bodyA0;
	public Body bodyA1;
	
	//Matches bodyD
	public Body bodyB0;
	public Body bodyB1;
	
	//Used to calculate restitution vs. each other
	
	public Body bodyC0;
	public Body bodyC1;
	
	public Body bodyD0;
	public Body bodyD1;

	float initV = 5.0f;
	float bodyLength = 3.0f;
	float springConstant = 500.0f;
	float springFrictionA = 00.0f;
	float springFrictionB = 200.0f;
	
	Vec2 ZEROVEC = new Vec2(0.0f,0.0f);
	
	private boolean firstTime = true;
	
    public SpringRestitutionTest(TestbedMain t) {
        super(t);
    }
    
    public String getName() {
    	return "Restitution Combination Test";
    }
    
    public void createWorld() {
		m_worldAABB = new AABB();
		m_worldAABB.lowerBound = new Vec2(-200.0f, -100.0f);
		m_worldAABB.upperBound = new Vec2(200.0f, 200.0f);
		// Set gravity to zero for this test
		Vec2 gravity = new Vec2(0.0f, -0.0f);
		boolean doSleep = true;
		m_world = new World(m_worldAABB, gravity, doSleep);
	}
	
    @Override
    public void create() {
    	if (firstTime) {
			setCamera(2f, 12f, 10f);
			firstTime = false;
		}
    	
    	{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(50.0f, 10.0f);

			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, -10.0f);
			Body ground = m_world.createBody(bd);
			ground.createShape(sd);
		}

    	
		{
			CircleDef sd = new CircleDef();
			sd.radius = 0.5f;
			sd.density = 5.0f;
			sd.restitution = 0.0f;
			sd.friction = 0.9f;
			
			BodyDef bd = new BodyDef();
			bd.position.set(-3.0f,10.0f);
			bodyA0 = m_world.createBody(bd);
			bodyA0.createShape(sd);
			bodyA0.setMassFromShapes();
			bodyA0.setLinearVelocity(new Vec2(0.0f, -initV));
			bd.position.y -= bodyLength;
			bodyA1 = m_world.createBody(bd);
			bodyA1.createShape(sd);
			bodyA1.setLinearVelocity(new Vec2(0.0f, -initV));
			bodyA1.setMassFromShapes();
			
			bd.position.set(-1.0f,10.0f);
			bodyB0 = m_world.createBody(bd);
			bodyB0.createShape(sd);
			bodyB0.setMassFromShapes();
			bodyB0.setLinearVelocity(new Vec2(0.0f, -initV));
			bd.position.y -= bodyLength;
			bodyB1 = m_world.createBody(bd);
			bodyB1.createShape(sd);
			bodyB1.setLinearVelocity(new Vec2(0.0f, -initV));
			bodyB1.setMassFromShapes();
			
			bd.position.set(1.0f,20.0f);
			bodyC0 = m_world.createBody(bd);
			bodyC0.createShape(sd);
			bodyC0.setMassFromShapes();
			bodyC0.setLinearVelocity(new Vec2(0.0f, -initV));
			bd.position.y -= bodyLength;
			bodyC1 = m_world.createBody(bd);
			bodyC1.createShape(sd);
			bodyC1.setLinearVelocity(new Vec2(0.0f, -initV));
			bodyC1.setMassFromShapes();
			
			bd.position.set(1.0f,5.0f);
			bodyD0 = m_world.createBody(bd);
			bodyD0.createShape(sd);
			bodyD0.setMassFromShapes();
			bodyD0.setLinearVelocity(new Vec2(0.0f, initV));
			bd.position.y -= bodyLength;
			bodyD1 = m_world.createBody(bd);
			bodyD1.createShape(sd);
			bodyD1.setLinearVelocity(new Vec2(0.0f, initV));
			bodyD1.setMassFromShapes();

		}
    }
    
    public void preStep() {
    	addSpringForce(bodyA0,ZEROVEC,bodyA1,ZEROVEC,springConstant,springFrictionA,bodyLength);
    	addSpringForce(bodyB0,ZEROVEC,bodyB1,ZEROVEC,springConstant,springFrictionB,bodyLength);
    	addSpringForce(bodyC0,ZEROVEC,bodyC1,ZEROVEC,springConstant,springFrictionA,bodyLength);
    	addSpringForce(bodyD0,ZEROVEC,bodyD1,ZEROVEC,springConstant,springFrictionB,bodyLength);
    }
    
    public void postStep() {
    	float finalVA = .5f*Math.abs(bodyA0.getLinearVelocity().y + bodyA1.getLinearVelocity().y);
    	float finalCORA = finalVA / initV;
    	m_debugDraw.drawString(5.0f,m_textLine,"BodyA: initial = "+initV+" , final = "+finalVA+"; COR A = "+finalCORA,white);
    	m_textLine += textLineHeight;
    	float finalVB = .5f*Math.abs(bodyB0.getLinearVelocity().y + bodyB1.getLinearVelocity().y);
    	float finalCORB = finalVB / initV;
    	m_debugDraw.drawString(5.0f,m_textLine,"BodyB: initial = "+initV+" , final = "+finalVB+"; COR B = "+finalCORB,white);
    	m_textLine += textLineHeight;
    	float finalVC = .5f*Math.abs(bodyC0.getLinearVelocity().y + bodyC1.getLinearVelocity().y);
    	float finalCORC = finalVC / initV;
    	m_debugDraw.drawString(5.0f,m_textLine,"BodyC: initial = "+initV+" , final = "+finalVC+"; COR C = "+finalCORC,white);
    	m_textLine += textLineHeight;
    	float finalVD = .5f*Math.abs(bodyD0.getLinearVelocity().y + bodyD1.getLinearVelocity().y);
    	float finalCORD = finalVD / initV;
    	m_debugDraw.drawString(5.0f,m_textLine,"BodyD: initial = "+initV+" , final = "+finalVD+"; COR D = "+finalCORD,white);
    	m_textLine += textLineHeight;
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
        diff.mulLocal(forceMag); // diff *= forceMag
        bB.applyForce(diff, bA.getWorldPoint(localA));
        bA.applyForce(diff.mulLocal(-1f), bB.getWorldPoint(localB));
        bA.wakeUp();
        bB.wakeUp();
    }
}

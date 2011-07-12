/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DANIEL MURPHY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
/**
 * Created at 7:50:04 AM Jan 20, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.FrictionJointDef;
import org.jbox2d.testbed.framework.TestPanel;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class ApplyForce extends TestbedTest {
	
	Body m_body;
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#initTest()
	 */
	@Override
	public void initTest() {
		m_world.setGravity(new Vec2(0.0f, 0.0f));
		
		final float k_restitution = 0.4f;
		
		Body ground;
		{
			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, 20.0f);
			ground = m_world.createBody(bd);
			
			PolygonShape shape = new PolygonShape();
			
			FixtureDef sd = new FixtureDef();
			sd.shape = shape;
			sd.density = 0.0f;
			sd.restitution = k_restitution;
			
			// Left vertical
			shape.setAsEdge(new Vec2(-20.0f, -20.0f), new Vec2(-20.0f, 20.0f));
			ground.createFixture(sd);
			
			// Right vertical
			shape.setAsEdge(new Vec2(20.0f, -20.0f), new Vec2(20.0f, 20.0f));
			ground.createFixture(sd);
			
			// Top horizontal
			shape.setAsEdge(new Vec2(-20.0f, 20.0f), new Vec2(20.0f, 20.0f));
			ground.createFixture(sd);
			
			// Bottom horizontal
			shape.setAsEdge(new Vec2(-20.0f, -20.0f), new Vec2(20.0f, -20.0f));
			ground.createFixture(sd);
		}
		
		{
			Transform xf1 = new Transform();
			xf1.R.set(0.3524f * MathUtils.PI);
			Mat22.mulToOut(xf1.R, new Vec2(1.0f, 0.0f), xf1.position);
			
			Vec2 vertices[] = new Vec2[3];
			vertices[0] = Transform.mul(xf1, new Vec2(-1.0f, 0.0f));
			vertices[1] = Transform.mul(xf1, new Vec2(1.0f, 0.0f));
			vertices[2] = Transform.mul(xf1, new Vec2(0.0f, 0.5f));
			
			PolygonShape poly1 = new PolygonShape();
			poly1.set(vertices, 3);
			
			FixtureDef sd1 = new FixtureDef();
			sd1.shape = poly1;
			sd1.density = 4.0f;
			
			Transform xf2 = new Transform();
			xf2.R.set(-0.3524f * MathUtils.PI);
			Mat22.mulToOut(xf2.R, new Vec2(-1.0f, 0.0f), xf2.position);
			
			vertices[0] = Transform.mul(xf2, new Vec2(-1.0f, 0.0f));
			vertices[1] = Transform.mul(xf2, new Vec2(1.0f, 0.0f));
			vertices[2] = Transform.mul(xf2, new Vec2(0.0f, 0.5f));
			
			PolygonShape poly2 = new PolygonShape();
			poly2.set(vertices, 3);
			
			FixtureDef sd2 = new FixtureDef();
			sd2.shape = poly2;
			sd2.density = 2.0f;
			
			BodyDef bd = new BodyDef();
			bd.type = BodyType.DYNAMIC;
			bd.angularDamping = 5.0f;
			bd.linearDamping = 0.1f;
			
			bd.position.set(0.0f, 2.0f);
			bd.angle = MathUtils.PI;
			bd.allowSleep = false;
			m_body = m_world.createBody(bd);
			m_body.createFixture(sd1);
			m_body.createFixture(sd2);
		}
		
		{
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(0.5f, 0.5f);
			
			FixtureDef fd = new FixtureDef();
			fd.shape = shape;
			fd.density = 1.0f;
			fd.friction = 0.3f;
			
			for (int i = 0; i < 10; ++i) {
				BodyDef bd = new BodyDef();
				bd.type = BodyType.DYNAMIC;
				
				bd.position.set(0.0f, 5.0f + 1.54f * i);
				Body body = m_world.createBody(bd);
				
				body.createFixture(fd);
				
				float gravity = 10.0f;
				float I = body.getInertia();
				float mass = body.getMass();
				
				// For a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
				float radius = MathUtils.sqrt(2.0f * I / mass);
				
				FrictionJointDef jd = new FrictionJointDef();
				jd.localAnchorA.setZero();
				jd.localAnchorB.setZero();
				jd.bodyA = ground;
				jd.bodyB = body;
				jd.collideConnected = true;
				jd.maxForce = mass * gravity;
				jd.maxTorque = mass * radius * gravity;
				
				m_world.createJoint(jd);
			}
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#step(org.jbox2d.testbed.framework.TestbedSettings)
	 */
	@Override
	public void step(TestbedSettings settings) {
		super.step(settings);
		
		addTextLine("Use 'wasd' to move, 'e' and 's' drift.");
		if(TestPanel.keys['w']){
			Vec2 f = m_body.getWorldVector(new Vec2(0.0f, -30.0f));
			Vec2 p = m_body.getWorldPoint(m_body.getLocalCenter().add(new Vec2(0.0f, 2.0f)));
			m_body.applyForce(f, p);
		}
		else if(TestPanel.keys['q']){
			Vec2 f = m_body.getWorldVector(new Vec2(0.0f, -30.0f));
			Vec2 p = m_body.getWorldPoint(m_body.getLocalCenter().add(new Vec2(-.2f, 0f)));
			m_body.applyForce(f, p);
		}
		else if(TestPanel.keys['e']){
			Vec2 f = m_body.getWorldVector(new Vec2(0.0f, -30.0f));
			Vec2 p = m_body.getWorldPoint(m_body.getLocalCenter().add(new Vec2(.2f, 0f)));
			m_body.applyForce(f, p);
		}
		else if(TestPanel.keys['s']){
			Vec2 f = m_body.getWorldVector(new Vec2(0.0f, 30.0f));
			Vec2 p = m_body.getWorldCenter();
			m_body.applyForce(f, p);
		}
		
		if(TestPanel.keys['a']){
			m_body.applyTorque(20.0f);
		}
		
		if(TestPanel.keys['d']){
			m_body.applyTorque(-20.0f);
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "Apply Force";
	}
	
}

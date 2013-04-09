/*******************************************************************************
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
/**
 * Created at 7:59:38 PM Jan 12, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class RevoluteTest extends TestbedTest {
	
	private RevoluteJoint m_joint;
	private boolean isLeft = false;
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#initTest(boolean)
	 */
	@Override
	public void initTest(boolean argDeserialized) {
		Body ground = null;
		{
			BodyDef bd = new BodyDef();
			ground = getWorld().createBody(bd);
			
			PolygonShape shape = new PolygonShape();
			shape.setAsEdge(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
			ground.createFixture(shape, 0.0f);
		}
		
		{
			CircleShape shape = new CircleShape();
			shape.m_radius = 0.5f;
			
			BodyDef bd = new BodyDef();
			bd.type = BodyType.DYNAMIC;
			
			RevoluteJointDef rjd = new RevoluteJointDef();
			
			bd.position.set(0.0f, 20.0f);
			Body body = getWorld().createBody(bd);
			body.createFixture(shape, 5.0f);
			
			float w = 100.0f;
			body.setAngularVelocity(w);
			body.setLinearVelocity(new Vec2(-8.0f * w, 0.0f));
			
			rjd.initialize(ground, body, new Vec2(0.0f, 12.0f));
			rjd.motorSpeed = -1.0f * MathUtils.PI;
			rjd.maxMotorTorque = 10000.0f;
			rjd.enableMotor = false;
			rjd.lowerAngle = -0.25f * MathUtils.PI;
			rjd.upperAngle = 0.5f * MathUtils.PI;
			rjd.enableLimit = true;
			rjd.collideConnected = true;
			
			m_joint = (RevoluteJoint) getWorld().createJoint(rjd);
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#step(org.jbox2d.testbed.framework.TestbedSettings)
	 */
	@Override
	public void step(TestbedSettings settings) {
		super.step(settings);
		addTextLine("Limits " + (m_joint.isLimitEnabled() ? "on" : "off") + ", Motor "
				+ (m_joint.isMotorEnabled() ? "on " : "off ") + (isLeft ? "left" : "right"));
		addTextLine("Keys: (l) limits, (m) motor, (a) left, (d) right");
		
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#keyPressed(char, int)
	 */
	@Override
	public void keyPressed(char argKeyChar, int argKeyCode) {
		
		switch (argKeyChar) {
			case 'l' :
				m_joint.enableLimit(!m_joint.isLimitEnabled());
				getModel().getKeys()['l'] = false;
				break;
			case 'm' :
				m_joint.enableMotor(!m_joint.isMotorEnabled());
				getModel().getKeys()['m'] = false;
				break;
			case 'a' :
				m_joint.setMotorSpeed(1.0f * MathUtils.PI);
				getModel().getKeys()['a'] = false;
				isLeft = true;
				break;
			case 'd' :
				m_joint.setMotorSpeed(-1.0f * MathUtils.PI);
				getModel().getKeys()['d'] = false;
				isLeft = false;
				break;
		}
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "Revolute";
	}
	
}

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
 * Created at 12:46:04 PM Jan 23, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.PulleyJoint;
import org.jbox2d.dynamics.joints.PulleyJointDef;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class Pulleys extends TestbedTest {
  private static final long JOINT_TAG = 2;

  PulleyJoint m_joint1;  

  @Override
  public Long getTag(Joint joint) {
    if (joint == m_joint1)
      return JOINT_TAG;
    return super.getTag(joint);
  }

  @Override
  public void processJoint(Joint joint, Long tag) {
    if (tag == JOINT_TAG) {
      m_joint1 = (PulleyJoint) joint;
    } else {
      super.processJoint(joint, tag);
    }
  }

  @Override
  public boolean isSaveLoadEnabled() {
    return true;
  }

  @Override
  public void initTest(boolean deserialized) {
    if (deserialized) {
      return;
    }
    float y = 16.0f;
    float L = 12.0f;
    float a = 1.0f;
    float b = 2.0f;
    Body ground = null;
    {
      BodyDef bd = new BodyDef();
      ground = getWorld().createBody(bd);

      EdgeShape shape = new EdgeShape();
      shape.set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
      ground.createFixture(shape, 0.0f);

      CircleShape circle = new CircleShape();
      circle.m_radius = 2.0f;

      circle.m_p.set(-10.0f, y + b + L);
      ground.createFixture(circle, 0.0f);

      circle.m_p.set(10.0f, y + b + L);
      ground.createFixture(circle, 0.0f);
    }

    {

      PolygonShape shape = new PolygonShape();
      shape.setAsBox(a, b);

      BodyDef bd = new BodyDef();
      bd.type = BodyType.DYNAMIC;

      bd.position.set(-10.0f, y);
      Body body1 = getWorld().createBody(bd);
      body1.createFixture(shape, 5.0f);

      bd.position.set(10.0f, y);
      Body body2 = getWorld().createBody(bd);
      body2.createFixture(shape, 5.0f);

      PulleyJointDef pulleyDef = new PulleyJointDef();
      Vec2 anchor1 = new Vec2(-10.0f, y + b);
      Vec2 anchor2 = new Vec2(10.0f, y + b);
      Vec2 groundAnchor1 = new Vec2(-10.0f, y + b + L);
      Vec2 groundAnchor2 = new Vec2(10.0f, y + b + L);
      pulleyDef.initialize(body1, body2, groundAnchor1, groundAnchor2, anchor1, anchor2, 2.0f);

      m_joint1 = (PulleyJoint) getWorld().createJoint(pulleyDef);
    }
  }

  @Override
  public void step(TestbedSettings settings) {
    super.step(settings);
    float ratio = m_joint1.getRatio();
    float L = m_joint1.getLength1() + ratio * m_joint1.getLength2();
    addTextLine("L1 + " + ratio + " * L2 = " + L);
    if (L >= 36) {
      addTextLine("Pulley is taught");
    }
  }

  @Override
  public String getTestName() {
    return "Pulleys";
  }

}

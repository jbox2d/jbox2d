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
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class RevoluteTest extends TestbedTest {
  private static final long JOINT_TAG = 1;
  private RevoluteJoint m_joint;
  private boolean isLeft = false;

  @Override
  public Long getTag(Joint joint) {
    if (joint == m_joint)
      return JOINT_TAG;
    return super.getTag(joint);
  }

  @Override
  public void processJoint(Joint joint, Long tag) {
    if (tag == JOINT_TAG) {
      m_joint = (RevoluteJoint) joint;
      isLeft = m_joint.getMotorSpeed() > 0;
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
    Body ground = null;
    {
      BodyDef bd = new BodyDef();
      ground = getWorld().createBody(bd);

      EdgeShape shape = new EdgeShape();
      shape.set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
      ground.createFixture(shape, 0.0f);
    }

    {
      CircleShape shape = new CircleShape();
      shape.m_radius = 0.5f;

      BodyDef bd = new BodyDef();
      bd.type = BodyType.DYNAMIC;

      RevoluteJointDef rjd = new RevoluteJointDef();

      bd.position.set(-10f, 20.0f);
      Body body = getWorld().createBody(bd);
      body.createFixture(shape, 5.0f);

      float w = 100.0f;
      body.setAngularVelocity(w);
      body.setLinearVelocity(new Vec2(-8.0f * w, 0.0f));

      rjd.initialize(ground, body, new Vec2(-10.0f, 12.0f));
      rjd.motorSpeed = -1.0f * MathUtils.PI;
      rjd.maxMotorTorque = 10000.0f;
      rjd.enableMotor = false;
      rjd.lowerAngle = -0.25f * MathUtils.PI;
      rjd.upperAngle = 0.5f * MathUtils.PI;
      rjd.enableLimit = true;
      rjd.collideConnected = true;

      m_joint = (RevoluteJoint) getWorld().createJoint(rjd);
    }

    {
      CircleShape circle_shape = new CircleShape();
      circle_shape.m_radius = 3.0f;

      BodyDef circle_bd = new BodyDef();
      circle_bd.type = BodyType.DYNAMIC;
      circle_bd.position.set(5.0f, 30.0f);

      FixtureDef fd = new FixtureDef();
      fd.density = 5.0f;
      fd.filter.maskBits = 1;
      fd.shape = circle_shape;

      Body ball = m_world.createBody(circle_bd);
      ball.createFixture(fd);

      PolygonShape polygon_shape = new PolygonShape();
      polygon_shape.setAsBox(10.0f, 0.2f, new Vec2(-10.0f, 0.0f), 0.0f);

      BodyDef polygon_bd = new BodyDef();
      polygon_bd.position.set(20.0f, 10.0f);
      polygon_bd.type = BodyType.DYNAMIC;
      polygon_bd.bullet = true;
      Body polygon_body = m_world.createBody(polygon_bd);
      polygon_body.createFixture(polygon_shape, 2.0f);

      RevoluteJointDef rjd = new RevoluteJointDef();
      rjd.initialize(ground, polygon_body, new Vec2(20.0f, 10.0f));
      rjd.lowerAngle = -0.25f * MathUtils.PI;
      rjd.upperAngle = 0.0f * MathUtils.PI;
      rjd.enableLimit = true;
      m_world.createJoint(rjd);
    }

    // Tests mass computation of a small object far from the origin
    {
      BodyDef bodyDef = new BodyDef();
      bodyDef.type = BodyType.DYNAMIC;
      Body body = m_world.createBody(bodyDef);

      PolygonShape polyShape = new PolygonShape();
      Vec2 verts[] = new Vec2[3];
      verts[0] = new Vec2(17.63f, 36.31f);
      verts[1] = new Vec2(17.52f, 36.69f);
      verts[2] = new Vec2(17.19f, 36.36f);
      polyShape.set(verts, 3);

      FixtureDef polyFixtureDef = new FixtureDef();
      polyFixtureDef.shape = polyShape;
      polyFixtureDef.density = 1;

      body.createFixture(polyFixtureDef); // assertion hits inside here
    }
  }

  @Override
  public void step(TestbedSettings settings) {
    super.step(settings);
    addTextLine("Limits " + (m_joint.isLimitEnabled() ? "on" : "off") + ", Motor "
        + (m_joint.isMotorEnabled() ? "on " : "off ") + (isLeft ? "left" : "right"));
    addTextLine("Keys: (l) limits, (m) motor, (a) left, (d) right");

  }

  @Override
  public void keyPressed(char argKeyChar, int argKeyCode) {
    switch (argKeyChar) {
      case 'l':
        m_joint.enableLimit(!m_joint.isLimitEnabled());
        break;
      case 'm':
        m_joint.enableMotor(!m_joint.isMotorEnabled());
        break;
      case 'a':
        m_joint.setMotorSpeed(1.0f * MathUtils.PI);
        isLeft = true;
        break;
      case 'd':
        m_joint.setMotorSpeed(-1.0f * MathUtils.PI);
        isLeft = false;
        break;
    }
  }

  @Override
  public String getTestName() {
    return "Revolute";
  }
}

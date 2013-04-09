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
 * Created at 8:02:54 PM Jan 23, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
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
public class CircleStress extends TestbedTest {

  private static final long JOINT_TAG = 1;

  private RevoluteJoint joint;

  @Override
  public Long getTag(Joint argJoint) {
    if (argJoint == joint) {
      return JOINT_TAG;
    }
    return null;
  }

  @Override
  public void processJoint(Joint argJoint, Long argTag) {
    if (argTag == JOINT_TAG) {
      joint = (RevoluteJoint) argJoint;
    }
  }

  @Override
  public boolean isSaveLoadEnabled() {
    return true;
  }

  @Override
  public Vec2 getDefaultCameraPos() {
    return new Vec2(0, 20);
  }

  @Override
  public float getDefaultCameraScale() {
    return 5;
  }

  @Override
  public void initTest(boolean argDeserialized) {
    if (argDeserialized) {
      return;
    }

    Body leftWall = null;
    Body rightWall = null;
    {
      // Ground
      PolygonShape sd = new PolygonShape();
      sd.setAsBox(50.0f, 10.0f);
      BodyDef bd = new BodyDef();
      bd.type = BodyType.STATIC;
      bd.position = new Vec2(0.0f, -10.0f);
      Body b = getWorld().createBody(bd);
      FixtureDef fd = new FixtureDef();
      fd.shape = sd;
      fd.friction = 1.0f;
      b.createFixture(fd);

      // Walls
      sd.setAsBox(3.0f, 50.0f);
      bd = new BodyDef();
      bd.position = new Vec2(45.0f, 25.0f);
      rightWall = getWorld().createBody(bd);
      rightWall.createFixture(sd, 0);
      bd.position = new Vec2(-45.0f, 25.0f);
      leftWall = getWorld().createBody(bd);
      leftWall.createFixture(sd, 0);

      // Corners
      bd = new BodyDef();
      sd.setAsBox(20.0f, 3.0f);
      bd.angle = (float) (-Math.PI / 4.0);
      bd.position = new Vec2(-35f, 8.0f);
      Body myBod = getWorld().createBody(bd);
      myBod.createFixture(sd, 0);
      bd.angle = (float) (Math.PI / 4.0);
      bd.position = new Vec2(35f, 8.0f);
      myBod = getWorld().createBody(bd);
      myBod.createFixture(sd, 0);

      // top
      sd.setAsBox(50.0f, 10.0f);
      bd.type = BodyType.STATIC;
      bd.angle = 0;
      bd.position = new Vec2(0.0f, 75.0f);
      b = getWorld().createBody(bd);
      fd.shape = sd;
      fd.friction = 1.0f;
      b.createFixture(fd);

    }

    CircleShape cd;
    FixtureDef fd = new FixtureDef();

    BodyDef bd = new BodyDef();
    bd.type = BodyType.DYNAMIC;
    int numPieces = 5;
    float radius = 6f;
    bd.position = new Vec2(0.0f, 10.0f);
    Body body = getWorld().createBody(bd);
    for (int i = 0; i < numPieces; i++) {
      cd = new CircleShape();
      cd.m_radius = 1.2f;
      fd.shape = cd;
      fd.density = 25;
      fd.friction = .1f;
      fd.restitution = .9f;
      float xPos = radius * (float) Math.cos(2f * Math.PI * (i / (float) (numPieces)));
      float yPos = radius * (float) Math.sin(2f * Math.PI * (i / (float) (numPieces)));
      cd.m_p.set(xPos, yPos);

      body.createFixture(fd);
    }

    body.setBullet(false);

    RevoluteJointDef rjd = new RevoluteJointDef();
    rjd.initialize(body, getGroundBody(), body.getPosition());
    rjd.motorSpeed = MathUtils.PI;
    rjd.maxMotorTorque = 1000000.0f;
    rjd.enableMotor = true;
    joint = (RevoluteJoint) getWorld().createJoint(rjd);

    {
      int loadSize = 41;

      for (int j = 0; j < 15; j++) {
        for (int i = 0; i < loadSize; i++) {
          CircleShape circ = new CircleShape();
          BodyDef bod = new BodyDef();
          bod.type = BodyType.DYNAMIC;
          circ.m_radius = 1.0f + (i % 2 == 0 ? 1.0f : -1.0f) * .5f * MathUtils.randomFloat(.5f, 1f);
          FixtureDef fd2 = new FixtureDef();
          fd2.shape = circ;
          fd2.density = circ.m_radius * 1.5f;
          fd2.friction = 0.5f;
          fd2.restitution = 0.7f;
          float xPos = -39f + 2 * i;
          float yPos = 50f + j;
          bod.position = new Vec2(xPos, yPos);
          Body myBody = getWorld().createBody(bod);
          myBody.createFixture(fd2);

        }
      }

    }

    getWorld().setGravity(new Vec2(0, -50));
  }

  @Override
  public void keyPressed(char argKeyChar, int argKeyCode) {
    switch (argKeyChar) {
      case 's':
        joint.setMotorSpeed(0);
        break;
      case '1':
        joint.setMotorSpeed(MathUtils.PI);
        break;
      case '2':
        joint.setMotorSpeed(MathUtils.PI * 2);
        break;
      case '3':
        joint.setMotorSpeed(MathUtils.PI * 3);
        break;
      case '4':
        joint.setMotorSpeed(MathUtils.PI * 6);
        break;
      case '5':
        joint.setMotorSpeed(MathUtils.PI * 10);
        break;
    }
  }

  @Override
  public void step(TestbedSettings settings) {
    // TODO Auto-generated method stub
    super.step(settings);

    addTextLine("Press 's' to stop, and '1' - '5' to change speeds");
  }

  @Override
  public String getTestName() {
    return "Circle Stress Test";
  }

}

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
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.framework.TestbedTest;

public class FreePendulumTest extends TestbedTest {
  private final boolean switchBodiesInJoint;

  public FreePendulumTest(boolean switchBodiesInJoint) {
    this.switchBodiesInJoint = switchBodiesInJoint;
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
    Body pendulum;
    Body base;
    Body ground;

    {
      CircleShape circleShape = new CircleShape();
      circleShape.m_radius = 1;
      Shape shape = circleShape;

      BodyDef bodyDef = new BodyDef();
      bodyDef.type = BodyType.DYNAMIC;
      bodyDef.position.set(-5, 0);
      bodyDef.allowSleep = false;
      pendulum = getWorld().createBody(bodyDef);
      pendulum.createFixture(shape, 1);
    }

    {
      PolygonShape shape = new PolygonShape();
      shape.setAsBox(1, 1);

      BodyDef bodyDef = new BodyDef();
      bodyDef.type = BodyType.DYNAMIC;
      bodyDef.position.set(0, 2);
      bodyDef.allowSleep = false;
      base = getWorld().createBody(bodyDef);
      base.createFixture(shape, 1);
    }

    {
      PolygonShape shape = new PolygonShape();
      shape.setAsBox(3, 1);

      BodyDef bodyDef = new BodyDef();
      bodyDef.type = BodyType.STATIC;
      ground = getWorld().createBody(bodyDef);
      ground.createFixture(shape, 0);
    }

    RevoluteJointDef jointDef = new RevoluteJointDef();

    if (switchBodiesInJoint)
      jointDef.initialize(pendulum, base, new Vec2(0, 0));
    else
      jointDef.initialize(base, pendulum, new Vec2(0, 0));

    getWorld().createJoint(jointDef);
  }

  @Override
  public String getTestName() {
    return "Free Pendulum " + (switchBodiesInJoint ? "1" : "0");
  }
}

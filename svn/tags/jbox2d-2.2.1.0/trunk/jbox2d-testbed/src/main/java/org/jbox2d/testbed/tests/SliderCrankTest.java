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
 * Created at 7:47:37 PM Jan 12, 2011
 */
package org.jbox2d.testbed.tests;

import java.util.Formatter;

import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.joints.PrismaticJoint;
import org.jbox2d.dynamics.joints.PrismaticJointDef;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class SliderCrankTest extends TestbedTest {

  private RevoluteJoint m_joint1;
  private PrismaticJoint m_joint2;

  @Override
  public void initTest(boolean argDeserialized) {
    Body ground = null;
    {
      BodyDef bd = new BodyDef();
      ground = getWorld().createBody(bd);

      EdgeShape shape = new EdgeShape();
      shape.set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
      ground.createFixture(shape, 0.0f);
    }

    {
      Body prevBody = ground;

      // Define crank.
      {
        PolygonShape shape = new PolygonShape();
        shape.setAsBox(0.5f, 2.0f);

        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(0.0f, 7.0f);
        Body body = getWorld().createBody(bd);
        body.createFixture(shape, 2.0f);

        RevoluteJointDef rjd = new RevoluteJointDef();
        rjd.initialize(prevBody, body, new Vec2(0.0f, 5.0f));
        rjd.motorSpeed = 1.0f * MathUtils.PI;
        rjd.maxMotorTorque = 10000.0f;
        rjd.enableMotor = true;
        m_joint1 = (RevoluteJoint) getWorld().createJoint(rjd);

        prevBody = body;
      }

      // Define follower.
      {
        PolygonShape shape = new PolygonShape();
        shape.setAsBox(0.5f, 4.0f);

        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(0.0f, 13.0f);
        Body body = getWorld().createBody(bd);
        body.createFixture(shape, 2.0f);

        RevoluteJointDef rjd = new RevoluteJointDef();
        rjd.initialize(prevBody, body, new Vec2(0.0f, 9.0f));
        rjd.enableMotor = false;
        getWorld().createJoint(rjd);

        prevBody = body;
      }

      // Define piston
      {
        PolygonShape shape = new PolygonShape();
        shape.setAsBox(1.5f, 1.5f);

        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.fixedRotation = true;
        bd.position.set(0.0f, 17.0f);
        Body body = getWorld().createBody(bd);
        body.createFixture(shape, 2.0f);

        RevoluteJointDef rjd = new RevoluteJointDef();
        rjd.initialize(prevBody, body, new Vec2(0.0f, 17.0f));
        getWorld().createJoint(rjd);

        PrismaticJointDef pjd = new PrismaticJointDef();
        pjd.initialize(ground, body, new Vec2(0.0f, 17.0f), new Vec2(0.0f, 1.0f));

        pjd.maxMotorForce = 1000.0f;
        pjd.enableMotor = false;

        m_joint2 = (PrismaticJoint) getWorld().createJoint(pjd);
      }

      // Create a payload
      {
        PolygonShape shape = new PolygonShape();
        shape.setAsBox(1.5f, 1.5f);

        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(0.0f, 23.0f);
        Body body = getWorld().createBody(bd);
        body.createFixture(shape, 2.0f);
      }
    }
  }

  @Override
  public void step(TestbedSettings settings) {
    super.step(settings);

    addTextLine("Keys: (f) toggle friction, (m) toggle motor");
    float torque = m_joint1.getMotorTorque(1);
    Formatter f = new Formatter();
    addTextLine(f.format("Friction: %b, Motor Force = %5.0f, ", m_joint2.isMotorEnabled(), torque)
        .toString());
    f.close();

  }

  @Override
  public void keyPressed(char argKeyChar, int argKeyCode) {

    switch (argKeyChar) {
      case 'f':
        m_joint2.enableMotor(!m_joint2.isMotorEnabled());
        getModel().getKeys()['f'] = false;
        break;
      case 'm':
        m_joint1.enableMotor(!m_joint1.isMotorEnabled());
        getModel().getKeys()['m'] = false;
        break;
    }
  }

  @Override
  public String getTestName() {
    return "Slider Crank";
  }
}

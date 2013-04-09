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
 * Created at 7:50:04 AM Jan 20, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Rot;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.FrictionJointDef;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class ApplyForce extends TestbedTest {
  private static final long BODY_TAG = 12;

  Body m_body;

  @Override
  public void initTest(boolean deserialized) {
    if (deserialized) {
      return;
    }

    getWorld().setGravity(new Vec2(0.0f, 0.0f));

    final float k_restitution = 0.4f;

    Body ground;
    {
      BodyDef bd = new BodyDef();
      bd.position.set(0.0f, 20.0f);
      ground = getWorld().createBody(bd);

      EdgeShape shape = new EdgeShape();

      FixtureDef sd = new FixtureDef();
      sd.shape = shape;
      sd.density = 0.0f;
      sd.restitution = k_restitution;

      // Left vertical
      shape.set(new Vec2(-20.0f, -20.0f), new Vec2(-20.0f, 20.0f));
      ground.createFixture(sd);

      // Right vertical
      shape.set(new Vec2(20.0f, -20.0f), new Vec2(20.0f, 20.0f));
      ground.createFixture(sd);

      // Top horizontal
      shape.set(new Vec2(-20.0f, 20.0f), new Vec2(20.0f, 20.0f));
      ground.createFixture(sd);

      // Bottom horizontal
      shape.set(new Vec2(-20.0f, -20.0f), new Vec2(20.0f, -20.0f));
      ground.createFixture(sd);
    }

    {
      Transform xf1 = new Transform();
      xf1.q.set(0.3524f * MathUtils.PI);
      Rot.mulToOutUnsafe(xf1.q, new Vec2(1.0f, 0.0f), xf1.p);

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
      xf2.q.set(-0.3524f * MathUtils.PI);
      Rot.mulToOut(xf2.q, new Vec2(-1.0f, 0.0f), xf2.p);

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
      m_body = getWorld().createBody(bd);
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
        Body body = getWorld().createBody(bd);

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

        getWorld().createJoint(jd);
      }
    }
  }

  @Override
  public void keyPressed(char keyCar, int keyCode) {
    // TODO Auto-generated method stub
    super.keyPressed(keyCar, keyCode);
  }
  
  @Override
  public void step(TestbedSettings settings) {
    super.step(settings);

    addTextLine("Use 'wasd' to move, 'e' and 's' drift.");
    if (getModel().getKeys()['w']) {
      Vec2 f = m_body.getWorldVector(new Vec2(0.0f, -30.0f));
      Vec2 p = m_body.getWorldPoint(m_body.getLocalCenter().add(new Vec2(0.0f, 2.0f)));
      m_body.applyForce(f, p);
    } else if (getModel().getKeys()['q']) {
      Vec2 f = m_body.getWorldVector(new Vec2(0.0f, -30.0f));
      Vec2 p = m_body.getWorldPoint(m_body.getLocalCenter().add(new Vec2(-.2f, 0f)));
      m_body.applyForce(f, p);
    } else if (getModel().getKeys()['e']) {
      Vec2 f = m_body.getWorldVector(new Vec2(0.0f, -30.0f));
      Vec2 p = m_body.getWorldPoint(m_body.getLocalCenter().add(new Vec2(.2f, 0f)));
      m_body.applyForce(f, p);
    } else if (getModel().getKeys()['s']) {
      Vec2 f = m_body.getWorldVector(new Vec2(0.0f, 30.0f));
      Vec2 p = m_body.getWorldCenter();
      m_body.applyForce(f, p);
    }

    if (getModel().getKeys()['a']) {
      m_body.applyTorque(20.0f);
    }

    if (getModel().getKeys()['d']) {
      m_body.applyTorque(-20.0f);
    }
  }

  @Override
  public boolean isSaveLoadEnabled() {
    return true;
  }

  @Override
  public Long getTag(Body body) {
    if (body == m_body) {
      return BODY_TAG;
    }
    return super.getTag(body);
  }

  @Override
  public void processBody(Body body, Long tag) {
    if (tag == BODY_TAG) {
      m_body = body;
    }
    super.processBody(body, tag);
  }

  @Override
  public String getTestName() {
    return "Apply Force";
  }
}

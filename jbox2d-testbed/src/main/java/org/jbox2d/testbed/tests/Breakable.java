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
 * Created at 5:18:10 AM Jan 14, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.callbacks.ContactImpulse;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class Breakable extends TestbedTest {

  Body m_body1;
  Vec2 m_velocity = new Vec2();
  float m_angularVelocity;
  PolygonShape m_shape1;
  PolygonShape m_shape2;
  Fixture m_piece1;
  Fixture m_piece2;

  boolean m_broke;
  boolean m_break;

  @Override
  public void initTest(boolean argDeserialized) {
    // Ground body
    {
      BodyDef bd = new BodyDef();
      Body ground = getWorld().createBody(bd);

      EdgeShape shape = new EdgeShape();
      shape.set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
      ground.createFixture(shape, 0.0f);
    }

    // Breakable dynamic body
    {
      BodyDef bd = new BodyDef();
      bd.type = BodyType.DYNAMIC;
      bd.position.set(0.0f, 40.0f);
      bd.angle = 0.25f * MathUtils.PI;
      m_body1 = getWorld().createBody(bd);

      m_shape1 = new PolygonShape();
      m_shape1.setAsBox(0.5f, 0.5f, new Vec2(-0.5f, 0.0f), 0.0f);
      m_piece1 = m_body1.createFixture(m_shape1, 1.0f);

      m_shape2 = new PolygonShape();
      m_shape2.setAsBox(0.5f, 0.5f, new Vec2(0.5f, 0.0f), 0.0f);
      m_piece2 = m_body1.createFixture(m_shape2, 1.0f);
    }

    m_break = false;
    m_broke = false;
  }

  @Override
  public void postSolve(Contact contact, ContactImpulse impulse) {
    if (m_broke) {
      // The body already broke.
      return;
    }

    // Should the body break?
    int count = contact.getManifold().pointCount;

    float maxImpulse = 0.0f;
    for (int i = 0; i < count; ++i) {
      maxImpulse = MathUtils.max(maxImpulse, impulse.normalImpulses[i]);
    }

    if (maxImpulse > 40.0f) {
      // Flag the body for breaking.
      m_break = true;
    }
  }

  void Break() {
    // Create two bodies from one.
    Body body1 = m_piece1.getBody();
    Vec2 center = body1.getWorldCenter();

    body1.destroyFixture(m_piece2);
    m_piece2 = null;

    BodyDef bd = new BodyDef();
    bd.type = BodyType.DYNAMIC;
    bd.position = body1.getPosition();
    bd.angle = body1.getAngle();

    Body body2 = getWorld().createBody(bd);
    m_piece2 = body2.createFixture(m_shape2, 1.0f);

    // Compute consistent velocities for new bodies based on
    // cached velocity.
    Vec2 center1 = body1.getWorldCenter();
    Vec2 center2 = body2.getWorldCenter();

    Vec2 velocity1 = m_velocity.add(Vec2.cross(m_angularVelocity, center1.sub(center)));
    Vec2 velocity2 = m_velocity.add(Vec2.cross(m_angularVelocity, center2.sub(center)));

    body1.setAngularVelocity(m_angularVelocity);
    body1.setLinearVelocity(velocity1);

    body2.setAngularVelocity(m_angularVelocity);
    body2.setLinearVelocity(velocity2);
  }

  @Override
  public void step(TestbedSettings settings) {
    super.step(settings);

    if (m_break) {
      Break();
      m_broke = true;
      m_break = false;
    }

    // Cache velocities to improve movement on breakage.
    if (m_broke == false) {
      m_velocity.set(m_body1.getLinearVelocity());
      m_angularVelocity = m_body1.getAngularVelocity();
    }
  }

  @Override
  public String getTestName() {
    return "Breakable";
  }
}

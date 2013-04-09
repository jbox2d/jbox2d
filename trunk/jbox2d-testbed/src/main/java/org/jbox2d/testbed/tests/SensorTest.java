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
 * Created at 1:25:51 PM Jan 23, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class SensorTest extends TestbedTest {

  class BoolWrapper {
    boolean tf;
  }

  int e_count = 7;
  Fixture m_sensor;
  Body m_bodies[] = new Body[e_count];
  BoolWrapper m_touching[] = new BoolWrapper[e_count];

  @Override
  public void initTest(boolean deserialized) {

    for (int i = 0; i < m_touching.length; i++) {
      m_touching[i] = new BoolWrapper();
    }

    {
      BodyDef bd = new BodyDef();
      Body ground = getWorld().createBody(bd);

      {
        EdgeShape shape = new EdgeShape();
        shape.set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
        ground.createFixture(shape, 0.0f);
      }

      {
        CircleShape shape = new CircleShape();
        shape.m_radius = 5.0f;
        shape.m_p.set(0.0f, 10.0f);

        FixtureDef fd = new FixtureDef();
        fd.shape = shape;
        fd.isSensor = true;
        m_sensor = ground.createFixture(fd);
      }
    }

    {
      CircleShape shape = new CircleShape();
      shape.m_radius = 1.0f;

      for (int i = 0; i < e_count; ++i) {
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(-10.0f + 3.0f * i, 20.0f);
        bd.userData = m_touching[i];

        m_touching[i].tf = false;
        m_bodies[i] = getWorld().createBody(bd);

        m_bodies[i].createFixture(shape, 1.0f);
      }
    }
  }

  // Implement contact listener.
  public void beginContact(Contact contact) {
    Fixture fixtureA = contact.getFixtureA();
    Fixture fixtureB = contact.getFixtureB();

    if (fixtureA == m_sensor) {
      Object userData = fixtureB.getBody().getUserData();
      if (userData != null) {
        ((BoolWrapper) userData).tf = true;
      }
    }

    if (fixtureB == m_sensor) {
      Object userData = fixtureA.getBody().getUserData();
      if (userData != null) {
        ((BoolWrapper) userData).tf = true;
      }
    }
  }

  // Implement contact listener.
  public void endContact(Contact contact) {
    Fixture fixtureA = contact.getFixtureA();
    Fixture fixtureB = contact.getFixtureB();

    if (fixtureA == m_sensor) {
      Object userData = fixtureB.getBody().getUserData();
      if (userData != null) {
        ((BoolWrapper) userData).tf = false;
      }
    }

    if (fixtureB == m_sensor) {
      Object userData = fixtureA.getBody().getUserData();
      if (userData != null) {
        ((BoolWrapper) userData).tf = false;
      }
    }
  }

  @Override
  public void step(TestbedSettings settings) {
    // TODO Auto-generated method stub
    super.step(settings);

    // Traverse the contact results. Apply a force on shapes
    // that overlap the sensor.
    for (int i = 0; i < e_count; ++i) {
      if (m_touching[i].tf == false) {
        continue;
      }

      Body body = m_bodies[i];
      Body ground = m_sensor.getBody();

      CircleShape circle = (CircleShape) m_sensor.getShape();
      Vec2 center = ground.getWorldPoint(circle.m_p);

      Vec2 position = body.getPosition();

      Vec2 d = center.sub(position);
      if (d.lengthSquared() < Settings.EPSILON * Settings.EPSILON) {
        continue;
      }

      d.normalize();
      Vec2 F = d.mulLocal(100f);
      body.applyForce(F, position);
    }
  }

  @Override
  public String getTestName() {
    return "Sensor Test";
  }
}

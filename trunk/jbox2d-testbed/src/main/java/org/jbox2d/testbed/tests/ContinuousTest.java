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
 * Created at 12:32:26 AM Aug 15, 2010
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.Distance;
import org.jbox2d.collision.TimeOfImpact;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class ContinuousTest extends TestbedTest {

  Body m_body;
  Fixture currFixture;
  PolygonShape m_poly;
  CircleShape m_circle;
  Shape nextShape = null;
  boolean polygon = false;
  float m_angularVelocity;

  @Override
  public String getTestName() {
    return "Continuous";
  }

  public void switchObjects() {
    if (polygon) {
      nextShape = m_circle;
    } else {
      nextShape = m_poly;
    }
    polygon = !polygon;
  }

  @Override
  public void initTest(boolean argDeserialized) {
    {
      BodyDef bd = new BodyDef();
      bd.position.set(0.0f, 0.0f);
      Body body = getWorld().createBody(bd);

      EdgeShape shape = new EdgeShape();

      shape.set(new Vec2(-10.0f, 0.0f), new Vec2(10.0f, 0.0f));
      body.createFixture(shape, 0.0f);

      PolygonShape pshape = new PolygonShape();
      pshape.setAsBox(0.2f, 1.0f, new Vec2(0.5f, 1.0f), 0.0f);
      body.createFixture(pshape, 0.0f);
    }
    m_poly = new PolygonShape();
    m_poly.setAsBox(2.0f, 0.1f);
    m_circle = new CircleShape();
    m_circle.m_p.setZero();
    m_circle.m_radius = 0.5f;

    BodyDef bd = new BodyDef();
    bd.type = BodyType.DYNAMIC;
    bd.position.set(0.0f, 20.0f);

    m_body = getWorld().createBody(bd);
    currFixture = m_body.createFixture(m_poly, 1.0f);

    m_angularVelocity = (float) Math.random() * 100 - 50;
    m_angularVelocity = 33.468121f;
    m_body.setLinearVelocity(new Vec2(0.0f, -100.0f));
    m_body.setAngularVelocity(m_angularVelocity);

    TimeOfImpact.toiCalls = 0;
    TimeOfImpact.toiIters = 0;
    TimeOfImpact.toiMaxIters = 0;
    TimeOfImpact.toiRootIters = 0;
    TimeOfImpact.toiMaxRootIters = 0;
  }

  public void launch() {
    m_body.setTransform(new Vec2(0.0f, 20.0f), 0.0f);
    m_angularVelocity = (float) Math.random() * 100 - 50;
    m_body.setLinearVelocity(new Vec2(0.0f, -100.0f));
    m_body.setAngularVelocity(m_angularVelocity);
  }

  @Override
  public void step(TestbedSettings settings) {
    if (nextShape != null) {
      m_body.destroyFixture(currFixture);
      currFixture = m_body.createFixture(nextShape, 1f);
      nextShape = null;
    }
    // if (stepCount == 12){
    // stepCount += 0;
    // } what is this?

    super.step(settings);

    if (Distance.GJK_CALLS > 0) {
      addTextLine(String.format("gjk calls = %d, ave gjk iters = %3.1f, max gjk iters = %d",
          Distance.GJK_CALLS, Distance.GJK_ITERS * 1. / Distance.GJK_CALLS, Distance.GJK_MAX_ITERS));
    }

    if (TimeOfImpact.toiCalls > 0) {
      int toiCalls = TimeOfImpact.toiCalls;
      int toiIters = TimeOfImpact.toiIters;
      int toiMaxIters = TimeOfImpact.toiMaxIters;
      int toiRootIters = TimeOfImpact.toiRootIters;
      int toiMaxRootIters = TimeOfImpact.toiMaxRootIters;
      addTextLine(String.format("toi calls = %d, ave toi iters = %3.1f, max toi iters = %d",
          toiCalls, toiIters * 1. / toiCalls, toiMaxIters));

      addTextLine(String.format("ave toi root iters = %3.1f, max toi root iters = %d", toiRootIters
          * 1. / toiCalls, toiMaxRootIters));
    }

    addTextLine("Press 'c' to change launch shape");

    if (getStepCount() % 60 == 0) {
      launch();
    }
  }

  @Override
  public void keyPressed(char argKeyChar, int argKeyCode) {
    switch (argKeyChar) {
      case 'c':
        switchObjects();
        break;
    }
  }
}

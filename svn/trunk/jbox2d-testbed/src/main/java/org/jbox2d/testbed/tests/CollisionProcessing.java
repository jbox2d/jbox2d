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
 * Created at 2:51:18 PM Jan 23, 2011
 */
package org.jbox2d.testbed.tests;

import java.util.HashSet;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.testbed.framework.ContactPoint;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class CollisionProcessing extends TestbedTest {

  @Override
  public boolean isSaveLoadEnabled() {
    return true;
  }

  @Override
  public void initTest(boolean deserialized) {
    if (deserialized) {
      return;
    }
    // Ground body
    {
      EdgeShape shape = new EdgeShape();
      shape.set(new Vec2(-50.0f, 0.0f), new Vec2(50.0f, 0.0f));

      FixtureDef sd = new FixtureDef();
      sd.shape = shape;

      BodyDef bd = new BodyDef();
      Body ground = getWorld().createBody(bd);
      ground.createFixture(sd);
    }

    float xLo = -5.0f, xHi = 5.0f;
    float yLo = 2.0f, yHi = 35.0f;

    // Small triangle
    Vec2 vertices[] = new Vec2[3];
    vertices[0] = new Vec2(-1.0f, 0.0f);
    vertices[1] = new Vec2(1.0f, 0.0f);
    vertices[2] = new Vec2(0.0f, 2.0f);

    PolygonShape polygon = new PolygonShape();
    polygon.set(vertices, 3);

    FixtureDef triangleShapeDef = new FixtureDef();
    triangleShapeDef.shape = polygon;
    triangleShapeDef.density = 1.0f;

    BodyDef triangleBodyDef = new BodyDef();
    triangleBodyDef.type = BodyType.DYNAMIC;
    triangleBodyDef.position.set(MathUtils.randomFloat(xLo, xHi), MathUtils.randomFloat(yLo, yHi));

    Body body1 = getWorld().createBody(triangleBodyDef);
    body1.createFixture(triangleShapeDef);

    // Large triangle (recycle definitions)
    vertices[0].mulLocal(2.0f);
    vertices[1].mulLocal(2.0f);
    vertices[2].mulLocal(2.0f);
    polygon.set(vertices, 3);

    triangleBodyDef.position.set(MathUtils.randomFloat(xLo, xHi), MathUtils.randomFloat(yLo, yHi));

    Body body2 = getWorld().createBody(triangleBodyDef);
    body2.createFixture(triangleShapeDef);

    // Small box
    polygon.setAsBox(1.0f, 0.5f);

    FixtureDef boxShapeDef = new FixtureDef();
    boxShapeDef.shape = polygon;
    boxShapeDef.density = 1.0f;

    BodyDef boxBodyDef = new BodyDef();
    boxBodyDef.type = BodyType.DYNAMIC;
    boxBodyDef.position.set(MathUtils.randomFloat(xLo, xHi), MathUtils.randomFloat(yLo, yHi));

    Body body3 = getWorld().createBody(boxBodyDef);
    body3.createFixture(boxShapeDef);

    // Large box (recycle definitions)
    polygon.setAsBox(2.0f, 1.0f);
    boxBodyDef.position.set(MathUtils.randomFloat(xLo, xHi), MathUtils.randomFloat(yLo, yHi));

    Body body4 = getWorld().createBody(boxBodyDef);
    body4.createFixture(boxShapeDef);

    // Small circle
    CircleShape circle = new CircleShape();
    circle.m_radius = 1.0f;

    FixtureDef circleShapeDef = new FixtureDef();
    circleShapeDef.shape = circle;
    circleShapeDef.density = 1.0f;

    BodyDef circleBodyDef = new BodyDef();
    circleBodyDef.type = BodyType.DYNAMIC;
    circleBodyDef.position.set(MathUtils.randomFloat(xLo, xHi), MathUtils.randomFloat(yLo, yHi));

    Body body5 = getWorld().createBody(circleBodyDef);
    body5.createFixture(circleShapeDef);

    // Large circle
    circle.m_radius *= 2.0f;
    circleBodyDef.position.set(MathUtils.randomFloat(xLo, xHi), MathUtils.randomFloat(yLo, yHi));

    Body body6 = getWorld().createBody(circleBodyDef);
    body6.createFixture(circleShapeDef);
  }

  @Override
  public void step(TestbedSettings settings) {
    super.step(settings);

    // We are going to destroy some bodies according to contact
    // points. We must buffer the bodies that should be destroyed
    // because they may belong to multiple contact points.
    HashSet<Body> nuke = new HashSet<Body>();

    // Traverse the contact results. Destroy bodies that
    // are touching heavier bodies.
    for (int i = 0; i < getPointCount(); ++i) {
      ContactPoint point = points[i];

      Body body1 = point.fixtureA.getBody();
      Body body2 = point.fixtureB.getBody();
      float mass1 = body1.getMass();
      float mass2 = body2.getMass();

      if (mass1 > 0.0f && mass2 > 0.0f) {
        if (mass2 > mass1) {
          nuke.add(body1);
        } else {
          nuke.add(body2);
        }
      }
    }

    // Sort the nuke array to group duplicates.
    // Arrays.sort(nuke);

    // Destroy the bodies, skipping duplicates.
    for (Body b : nuke) {

      if (b != getBomb()) {
        getWorld().destroyBody(b);
      }
    }
  }

  @Override
  public String getTestName() {
    return "Collision Processing";
  }
}

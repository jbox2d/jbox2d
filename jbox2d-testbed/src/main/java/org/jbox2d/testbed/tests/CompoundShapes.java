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
 * Created at 1:13:43 AM Sep 3, 2010
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Rot;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class CompoundShapes extends TestbedTest {

  @Override
  public boolean isSaveLoadEnabled() {
    return true;
  }

  @Override
  public void initTest(boolean deserialized) {
    if (deserialized) {
      return;
    }

    {
      BodyDef bd = new BodyDef();
      bd.position.set(0.0f, 0.0f);
      Body body = getWorld().createBody(bd);

      EdgeShape shape = new EdgeShape();
      shape.set(new Vec2(50.0f, 0.0f), new Vec2(-50.0f, 0.0f));

      body.createFixture(shape, 0.0f);
    }

    {
      CircleShape circle1 = new CircleShape();
      circle1.m_radius = 0.5f;
      circle1.m_p.set(-0.5f, 0.5f);

      CircleShape circle2 = new CircleShape();;
      circle2.m_radius = 0.5f;
      circle2.m_p.set(0.5f, 0.5f);

      for (int i = 0; i < 10; ++i) {
        float x = MathUtils.randomFloat(-0.1f, 0.1f);
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(x + 5.0f, 1.05f + 2.5f * i);
        bd.angle = MathUtils.randomFloat(-MathUtils.PI, MathUtils.PI);
        Body body = getWorld().createBody(bd);
        body.createFixture(circle1, 2.0f);
        body.createFixture(circle2, 0.0f);
      }
    }

    {
      PolygonShape polygon1 = new PolygonShape();
      polygon1.setAsBox(0.25f, 0.5f);

      PolygonShape polygon2 = new PolygonShape();
      polygon2.setAsBox(0.25f, 0.5f, new Vec2(0.0f, -0.5f), 0.5f * MathUtils.PI);

      for (int i = 0; i < 10; ++i) {
        float x = MathUtils.randomFloat(-0.1f, 0.1f);
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(x - 5.0f, 1.05f + 2.5f * i);
        bd.angle = MathUtils.randomFloat(-MathUtils.PI, MathUtils.PI);
        Body body = getWorld().createBody(bd);
        body.createFixture(polygon1, 2.0f);
        body.createFixture(polygon2, 2.0f);
      }
    }

    {
      Transform xf1 = new Transform();
      xf1.q.set(0.3524f * MathUtils.PI);
      Rot.mulToOut(xf1.q, new Vec2(1.0f, 0.0f), xf1.p);

      Vec2[] vertices = new Vec2[3];

      PolygonShape triangle1 = new PolygonShape();
      vertices[0] = Transform.mul(xf1, new Vec2(-1.0f, 0.0f));
      vertices[1] = Transform.mul(xf1, new Vec2(1.0f, 0.0f));
      vertices[2] = Transform.mul(xf1, new Vec2(0.0f, 0.5f));
      triangle1.set(vertices, 3);

      Transform xf2 = new Transform();
      xf2.q.set(-0.3524f * MathUtils.PI);
      Rot.mulToOut(xf2.q, new Vec2(-1.0f, 0.0f), xf2.p);

      PolygonShape triangle2 = new PolygonShape();
      vertices[0] = Transform.mul(xf2, new Vec2(-1.0f, 0.0f));
      vertices[1] = Transform.mul(xf2, new Vec2(1.0f, 0.0f));
      vertices[2] = Transform.mul(xf2, new Vec2(0.0f, 0.5f));
      triangle2.set(vertices, 3);

      for (int i = 0; i < 10; ++i) {
        float x = MathUtils.randomFloat(-0.1f, 0.1f);
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(x, 2.05f + 2.5f * i);
        bd.angle = 0.0f;
        Body body = getWorld().createBody(bd);
        body.createFixture(triangle1, 2.0f);
        body.createFixture(triangle2, 2.0f);
      }
    }

    {
      PolygonShape bottom = new PolygonShape();
      bottom.setAsBox(1.5f, 0.15f);

      PolygonShape left = new PolygonShape();
      left.setAsBox(0.15f, 2.7f, new Vec2(-1.45f, 2.35f), 0.2f);

      PolygonShape right = new PolygonShape();
      right.setAsBox(0.15f, 2.7f, new Vec2(1.45f, 2.35f), -0.2f);

      BodyDef bd = new BodyDef();
      bd.type = BodyType.DYNAMIC;
      bd.position.set(0.0f, 2.0f);
      Body body = getWorld().createBody(bd);
      body.createFixture(bottom, 4.0f);
      body.createFixture(left, 4.0f);
      body.createFixture(right, 4.0f);
    }
  }

  @Override
  public String getTestName() {
    return "Compound Shapes";
  }
}

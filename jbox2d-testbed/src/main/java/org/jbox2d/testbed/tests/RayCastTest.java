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
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;
import org.jbox2d.callbacks.RayCastCallback;

public class RayCastTest extends TestbedTest {

  public static final int e_maxBodies = 256;

  enum Mode {
    e_closest, e_any, e_multiple
  };

  int m_bodyIndex;
  Body[] m_bodies;
  Integer[] m_userData;
  PolygonShape[] m_polygons;
  CircleShape m_circle;

  float m_angle;

  Mode m_mode;

  @Override
  public String getTestName() {
    return "Raycast";
  }

  @Override
  public void initTest(boolean deserialized) {
    m_bodies = new Body[e_maxBodies];
    m_userData = new Integer[e_maxBodies];
    m_polygons = new PolygonShape[4];
    {
      BodyDef bd = new BodyDef();
      Body ground = getWorld().createBody(bd);

      EdgeShape shape = new EdgeShape();
      shape.set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
      ground.createFixture(shape, 0.0f);
    }

    {
      Vec2 vertices[] = new Vec2[3];
      vertices[0] = new Vec2(-0.5f, 0.0f);
      vertices[1] = new Vec2(0.5f, 0.0f);
      vertices[2] = new Vec2(0.0f, 1.5f);
      m_polygons[0] = new PolygonShape();
      m_polygons[0].set(vertices, 3);
    }

    {
      Vec2 vertices[] = new Vec2[3];
      vertices[0] = new Vec2(-0.1f, 0.0f);
      vertices[1] = new Vec2(0.1f, 0.0f);
      vertices[2] = new Vec2(0.0f, 1.5f);
      m_polygons[1] = new PolygonShape();
      m_polygons[1].set(vertices, 3);
    }

    {
      float w = 1.0f;
      float b = w / (2.0f + MathUtils.sqrt(2.0f));
      float s = MathUtils.sqrt(2.0f) * b;

      Vec2 vertices[] = new Vec2[8];
      vertices[0] = new Vec2(0.5f * s, 0.0f);
      vertices[1] = new Vec2(0.5f * w, b);
      vertices[2] = new Vec2(0.5f * w, b + s);
      vertices[3] = new Vec2(0.5f * s, w);
      vertices[4] = new Vec2(-0.5f * s, w);
      vertices[5] = new Vec2(-0.5f * w, b + s);
      vertices[6] = new Vec2(-0.5f * w, b);
      vertices[7] = new Vec2(-0.5f * s, 0.0f);

      m_polygons[2] = new PolygonShape();
      m_polygons[2].set(vertices, 8);
    }

    {
      m_polygons[3] = new PolygonShape();
      m_polygons[3].setAsBox(0.5f, 0.5f);
    }

    {
      m_circle = new CircleShape();
      m_circle.m_radius = 0.5f;
    }

    m_bodyIndex = 0;

    m_angle = 0.0f;

    m_mode = Mode.e_closest;
  }

  RayCastClosestCallback ccallback = new RayCastClosestCallback();
  RayCastAnyCallback acallback = new RayCastAnyCallback();
  RayCastMultipleCallback mcallback = new RayCastMultipleCallback();

  // pooling
  Vec2 point1 = new Vec2();
  Vec2 d = new Vec2();
  Vec2 pooledHead = new Vec2();
  Vec2 point2 = new Vec2();

  @Override
  public void step(TestbedSettings settings) {
    boolean advanceRay = settings.pause == false || settings.singleStep;

    super.step(settings);

    addTextLine("Press 1-5 to drop stuff, m to change the mode");
    addTextLine("Polygon 1 is filtered");
    addTextLine("Mode = " + m_mode);

    float L = 11.0f;
    point1.set(0.0f, 10.0f);
    d.set(L * MathUtils.cos(m_angle), L * MathUtils.sin(m_angle));
    point2.set(point1);
    point2.addLocal(d);

    if (m_mode == Mode.e_closest) {
      ccallback.init();
      getWorld().raycast(ccallback, point1, point2);

      if (ccallback.m_hit) {
        getDebugDraw().drawPoint(ccallback.m_point, 5.0f, new Color3f(0.4f, 0.9f, 0.4f));
        getDebugDraw().drawSegment(point1, ccallback.m_point, new Color3f(0.8f, 0.8f, 0.8f));
        pooledHead.set(ccallback.m_normal);
        pooledHead.mulLocal(.5f).addLocal(ccallback.m_point);
        getDebugDraw().drawSegment(ccallback.m_point, pooledHead, new Color3f(0.9f, 0.9f, 0.4f));
      } else {
        getDebugDraw().drawSegment(point1, point2, new Color3f(0.8f, 0.8f, 0.8f));
      }
    } else if (m_mode == Mode.e_any) {
      acallback.init();
      getWorld().raycast(acallback, point1, point2);

      if (acallback.m_hit) {
        getDebugDraw().drawPoint(acallback.m_point, 5.0f, new Color3f(0.4f, 0.9f, 0.4f));
        getDebugDraw().drawSegment(point1, acallback.m_point, new Color3f(0.8f, 0.8f, 0.8f));
        pooledHead.set(acallback.m_normal);
        pooledHead.mulLocal(.5f).addLocal(acallback.m_point);
        getDebugDraw().drawSegment(acallback.m_point, pooledHead, new Color3f(0.9f, 0.9f, 0.4f));
      } else {
        getDebugDraw().drawSegment(point1, point2, new Color3f(0.8f, 0.8f, 0.8f));
      }
    } else if (m_mode == Mode.e_multiple) {
      mcallback.init();
      getWorld().raycast(mcallback, point1, point2);
      getDebugDraw().drawSegment(point1, point2, new Color3f(0.8f, 0.8f, 0.8f));

      for (int i = 0; i < mcallback.m_count; ++i) {
        Vec2 p = mcallback.m_points[i];
        Vec2 n = mcallback.m_normals[i];
        getDebugDraw().drawPoint(p, 5.0f, new Color3f(0.4f, 0.9f, 0.4f));
        getDebugDraw().drawSegment(point1, p, new Color3f(0.8f, 0.8f, 0.8f));
        pooledHead.set(n);
        pooledHead.mulLocal(.5f).addLocal(p);
        getDebugDraw().drawSegment(p, pooledHead, new Color3f(0.9f, 0.9f, 0.4f));
      }
    }

    if (advanceRay) {
      m_angle += 0.25f * MathUtils.PI / 180.0f;
    }
  }

  void Create(int index) {
    if (m_bodies[m_bodyIndex] != null) {
      getWorld().destroyBody(m_bodies[m_bodyIndex]);
      m_bodies[m_bodyIndex] = null;
    }

    BodyDef bd = new BodyDef();

    float x = (float) Math.random() * 20 - 10;
    float y = (float) Math.random() * 20;
    bd.position.set(x, y);
    bd.angle = (float) Math.random() * MathUtils.TWOPI - MathUtils.PI;

    m_userData[m_bodyIndex] = index;
    bd.userData = m_userData[m_bodyIndex];

    if (index == 4) {
      bd.angularDamping = 0.02f;
    }

    m_bodies[m_bodyIndex] = getWorld().createBody(bd);

    if (index < 4) {
      FixtureDef fd = new FixtureDef();
      fd.shape = m_polygons[index];
      fd.friction = 0.3f;
      m_bodies[m_bodyIndex].createFixture(fd);
    } else {
      FixtureDef fd = new FixtureDef();
      fd.shape = m_circle;
      fd.friction = 0.3f;

      m_bodies[m_bodyIndex].createFixture(fd);
    }

    m_bodyIndex = (m_bodyIndex + 1) % e_maxBodies;
  }

  void DestroyBody() {
    for (int i = 0; i < e_maxBodies; ++i) {
      if (m_bodies[i] != null) {
        getWorld().destroyBody(m_bodies[i]);
        m_bodies[i] = null;
        return;
      }
    }
  }

  @Override
  public void keyPressed(char argKeyChar, int argKeyCode) {

    switch (argKeyChar) {
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
        Create(argKeyChar - '1');
        break;

      case 'd':
        DestroyBody();
        break;

      case 'm':
        if (m_mode == Mode.e_closest) {
          m_mode = Mode.e_any;
        } else if (m_mode == Mode.e_any) {
          m_mode = Mode.e_multiple;
        } else if (m_mode == Mode.e_multiple) {
          m_mode = Mode.e_closest;
        }
        break;
    }
  }

}

// This test demonstrates how to use the world ray-cast feature.
// NOTE: we are intentionally filtering one of the polygons, therefore
// the ray will always miss one type of polygon.

// This callback finds the closest hit. Polygon 0 is filtered.
class RayCastClosestCallback implements RayCastCallback {

  boolean m_hit;
  Vec2 m_point;
  Vec2 m_normal;

  public void init() {
    m_hit = false;
  }

  public float reportFixture(Fixture fixture, Vec2 point, Vec2 normal, float fraction) {
    Body body = fixture.getBody();
    Object userData = body.getUserData();
    if (userData != null) {
      int index = (Integer) userData;
      if (index == 0) {
        // filter
        return -1f;
      }
    }

    m_hit = true;
    m_point = point;
    m_normal = normal;
    return fraction;
  }

};

// This callback finds any hit. Polygon 0 is filtered.
class RayCastAnyCallback implements RayCastCallback {
  public void init() {
    m_hit = false;
  }

  public float reportFixture(Fixture fixture, Vec2 point, Vec2 normal, float fraction) {
    Body body = fixture.getBody();
    Object userData = body.getUserData();
    if (userData != null) {
      int index = (Integer) userData;
      if (index == 0) {
        // filter
        return -1f;
      }
    }

    m_hit = true;
    m_point = point;
    m_normal = normal;
    return 0f;
  }

  boolean m_hit;
  Vec2 m_point;
  Vec2 m_normal;
};

// This ray cast collects multiple hits along the ray. Polygon 0 is filtered.
class RayCastMultipleCallback implements RayCastCallback {
  public int e_maxCount = 30;
  Vec2 m_points[] = new Vec2[e_maxCount];
  Vec2 m_normals[] = new Vec2[e_maxCount];
  int m_count;

  public void init() {
    for (int i = 0; i < e_maxCount; i++) {
      m_points[i] = new Vec2();
      m_normals[i] = new Vec2();
    }
    m_count = 0;
  }

  public float reportFixture(Fixture fixture, Vec2 point, Vec2 normal, float fraction) {
    Body body = fixture.getBody();
    int index = 0;
    Object userData = body.getUserData();
    if (userData != null) {
      index = (Integer) userData;
      if (index == 0) {
        // filter
        return -1f;
      }
    }

    assert (m_count < e_maxCount);

    m_points[m_count].set(point);
    m_normals[m_count].set(normal);
    ++m_count;

    if (m_count == e_maxCount) {
      return 0f;
    }

    return 1f;
  }

};

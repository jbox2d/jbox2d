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
 * Created at 4:11:55 AM Jan 15, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.WeldJointDef;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class Cantilever extends TestbedTest {

  int e_count = 8;

  @Override
  public boolean isSaveLoadEnabled() {
    return true;
  }

  @Override
  public void initTest(boolean argDeserialized) {
    if (argDeserialized) {
      return;
    }
    Body ground = null;
    {
      BodyDef bd = new BodyDef();
      ground = getWorld().createBody(bd);

      EdgeShape shape = new EdgeShape();
      shape.set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
      ground.createFixture(shape, 0.0f);
    }

    {
      PolygonShape shape = new PolygonShape();
      shape.setAsBox(0.5f, 0.125f);

      FixtureDef fd = new FixtureDef();
      fd.shape = shape;
      fd.density = 20.0f;

      WeldJointDef jd = new WeldJointDef();

      Body prevBody = ground;
      for (int i = 0; i < e_count; ++i) {
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(-14.5f + 1.0f * i, 5.0f);
        Body body = getWorld().createBody(bd);
        body.createFixture(fd);

        Vec2 anchor = new Vec2(-15.0f + 1.0f * i, 5.0f);
        jd.initialize(prevBody, body, anchor);
        getWorld().createJoint(jd);

        prevBody = body;
      }
    }

    {
      PolygonShape shape = new PolygonShape();
      shape.setAsBox(1f, 0.125f);

      FixtureDef fd = new FixtureDef();
      fd.shape = shape;
      fd.density = 20.0f;

      WeldJointDef jd = new WeldJointDef();
      jd.frequencyHz = 5f;
      jd.dampingRatio = .7f;

      Body prevBody = ground;
      for (int i = 0; i < 3; ++i) {
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(-14.0f + 2.0f * i, 15.0f);
        Body body = getWorld().createBody(bd);
        body.createFixture(fd);

        Vec2 anchor = new Vec2(-15.0f + 2.0f * i, 15.0f);
        jd.initialize(prevBody, body, anchor);
        getWorld().createJoint(jd);

        prevBody = body;
      }
    }

    {
      PolygonShape shape = new PolygonShape();
      shape.setAsBox(0.5f, 0.125f);

      FixtureDef fd = new FixtureDef();
      fd.shape = shape;
      fd.density = 20.0f;

      WeldJointDef jd = new WeldJointDef();

      Body prevBody = ground;
      for (int i = 0; i < e_count; ++i) {
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(-4.5f + 1.0f * i, 5.0f);
        Body body = getWorld().createBody(bd);
        body.createFixture(fd);

        if (i > 0) {
          Vec2 anchor = new Vec2(-5.0f + 1.0f * i, 5.0f);
          jd.initialize(prevBody, body, anchor);
          getWorld().createJoint(jd);
        }

        prevBody = body;
      }
    }

    {
      PolygonShape shape = new PolygonShape();
      shape.setAsBox(0.5f, 0.125f);

      FixtureDef fd = new FixtureDef();
      fd.shape = shape;
      fd.density = 20.0f;

      WeldJointDef jd = new WeldJointDef();
      jd.frequencyHz = 8f;
      jd.dampingRatio = .7f;

      Body prevBody = ground;
      for (int i = 0; i < e_count; ++i) {
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(5.5f + 1.0f * i, 10.0f);
        Body body = getWorld().createBody(bd);
        body.createFixture(fd);

        if (i > 0) {
          Vec2 anchor = new Vec2(5.0f + 1.0f * i, 10.0f);
          jd.initialize(prevBody, body, anchor);
          getWorld().createJoint(jd);
        }

        prevBody = body;
      }
    }

    for (int i = 0; i < 2; ++i) {
      Vec2 vertices[] = new Vec2[3];
      vertices[0] = new Vec2(-0.5f, 0.0f);
      vertices[1] = new Vec2(0.5f, 0.0f);
      vertices[2] = new Vec2(0.0f, 1.5f);

      PolygonShape shape = new PolygonShape();
      shape.set(vertices, 3);

      FixtureDef fd = new FixtureDef();
      fd.shape = shape;
      fd.density = 1.0f;

      BodyDef bd = new BodyDef();
      bd.type = BodyType.DYNAMIC;
      bd.position.set(-8.0f + 8.0f * i, 12.0f);
      Body body = getWorld().createBody(bd);
      body.createFixture(fd);
    }

    for (int i = 0; i < 2; ++i) {
      CircleShape shape = new CircleShape();
      shape.m_radius = 0.5f;

      FixtureDef fd = new FixtureDef();
      fd.shape = shape;
      fd.density = 1.0f;

      BodyDef bd = new BodyDef();
      bd.type = BodyType.DYNAMIC;
      bd.position.set(-6.0f + 6.0f * i, 10.0f);
      Body body = getWorld().createBody(bd);
      body.createFixture(fd);
    }
  }

  @Override
  public String getTestName() {
    return "Cantilever";
  }
}

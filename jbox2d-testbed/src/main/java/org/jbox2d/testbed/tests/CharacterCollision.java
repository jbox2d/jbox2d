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
 * Created at 2:39:05 PM Jan 23, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class CharacterCollision extends TestbedTest {

  @Override
  public boolean isSaveLoadEnabled() {
    return true;
  }

  /**
   * @see org.jbox2d.testbed.framework.TestbedTest#initTest(boolean)
   */
  @Override
  public void initTest(boolean argDeserialized) {
    // Ground body
    {
      BodyDef bd = new BodyDef();
      Body ground = getWorld().createBody(bd);

      PolygonShape shape = new PolygonShape();
      shape.setAsEdge(new Vec2(-20.0f, 0.0f), new Vec2(20.0f, 0.0f));
      ground.createFixture(shape, 0.0f);
    }

    // Collinear edges
    {
      BodyDef bd = new BodyDef();
      Body ground = getWorld().createBody(bd);

      PolygonShape shape = new PolygonShape();
      shape.m_radius = 0.0f;
      shape.setAsEdge(new Vec2(-8.0f, 1.0f), new Vec2(-6.0f, 1.0f));
      ground.createFixture(shape, 0.0f);
      shape.setAsEdge(new Vec2(-6.0f, 1.0f), new Vec2(-4.0f, 1.0f));
      ground.createFixture(shape, 0.0f);
      shape.setAsEdge(new Vec2(-4.0f, 1.0f), new Vec2(-2.0f, 1.0f));
      ground.createFixture(shape, 0.0f);
    }

    // Square tiles
    {
      BodyDef bd = new BodyDef();
      Body ground = getWorld().createBody(bd);

      PolygonShape shape = new PolygonShape();
      shape.setAsBox(1.0f, 1.0f, new Vec2(4.0f, 3.0f), 0.0f);
      ground.createFixture(shape, 0.0f);
      shape.setAsBox(1.0f, 1.0f, new Vec2(6.0f, 3.0f), 0.0f);
      ground.createFixture(shape, 0.0f);
      shape.setAsBox(1.0f, 1.0f, new Vec2(8.0f, 3.0f), 0.0f);
      ground.createFixture(shape, 0.0f);
    }

    // Square made from edges notice how the edges are shrunk to account
    // for the polygon radius. This makes it so the square character does
    // not get snagged. However, ray casts can now go through the cracks.
    {
      BodyDef bd = new BodyDef();
      Body ground = getWorld().createBody(bd);

      PolygonShape shape = new PolygonShape();
      float d = 2.0f * Settings.polygonRadius;
      shape.setAsEdge(new Vec2(-1.0f + d, 3.0f), new Vec2(1.0f - d, 3.0f));
      ground.createFixture(shape, 0.0f);
      shape.setAsEdge(new Vec2(1.0f, 3.0f + d), new Vec2(1.0f, 5.0f - d));
      ground.createFixture(shape, 0.0f);
      shape.setAsEdge(new Vec2(1.0f - d, 5.0f), new Vec2(-1.0f + d, 5.0f));
      ground.createFixture(shape, 0.0f);
      shape.setAsEdge(new Vec2(-1.0f, 5.0f - d), new Vec2(-1.0f, 3.0f + d));
      ground.createFixture(shape, 0.0f);
    }

    // Square character
    {
      BodyDef bd = new BodyDef();
      bd.position.set(-3.0f, 5.0f);
      bd.type = BodyType.DYNAMIC;
      bd.fixedRotation = true;
      bd.allowSleep = false;

      Body body = getWorld().createBody(bd);

      PolygonShape shape = new PolygonShape();
      shape.setAsBox(0.5f, 0.5f);

      FixtureDef fd = new FixtureDef();
      fd.shape = shape;
      fd.density = 20.0f;
      body.createFixture(fd);
    }

    // Hexagon character
    {
      BodyDef bd = new BodyDef();
      bd.position.set(-5.0f, 5.0f);
      bd.type = BodyType.DYNAMIC;
      bd.fixedRotation = true;
      bd.allowSleep = false;

      Body body = getWorld().createBody(bd);

      float angle = 0.0f;
      float delta = MathUtils.PI / 3.0f;
      Vec2 vertices[] = new Vec2[6];
      for (int i = 0; i < 6; ++i) {
        vertices[i] = new Vec2(0.5f * MathUtils.cos(angle), 0.5f * MathUtils.sin(angle));
        angle += delta;
      }

      PolygonShape shape = new PolygonShape();
      shape.set(vertices, 6);

      FixtureDef fd = new FixtureDef();
      fd.shape = shape;
      fd.density = 20.0f;
      body.createFixture(fd);
    }

    // Circle character
    {
      BodyDef bd = new BodyDef();
      bd.position.set(3.0f, 5.0f);
      bd.type = BodyType.DYNAMIC;
      bd.fixedRotation = true;
      bd.allowSleep = false;

      Body body = getWorld().createBody(bd);

      CircleShape shape = new CircleShape();
      shape.m_radius = 0.5f;

      FixtureDef fd = new FixtureDef();
      fd.shape = shape;
      fd.density = 20.0f;
      body.createFixture(fd);
    }
  }

  /**
   * @see org.jbox2d.testbed.framework.TestbedTest#step(org.jbox2d.testbed.framework.TestbedSettings)
   */
  @Override
  public void step(TestbedSettings settings) {
    super.step(settings);
    addTextLine("This tests various character collision shapes");
  }

  /**
   * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
   */
  @Override
  public String getTestName() {
    return "Character Collision";
  }

}

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

import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.testbed.framework.TestbedTest;

public class VaryingFrictionTest extends TestbedTest {

  @Override
  public String getTestName() {
    return "Varying Friction";
  }

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
      Body ground = getWorld().createBody(bd);

      EdgeShape shape = new EdgeShape();
      shape.set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
      ground.createFixture(shape, 0.0f);
    }

    {
      PolygonShape shape = new PolygonShape();
      shape.setAsBox(13.0f, 0.25f);

      BodyDef bd = new BodyDef();
      bd.position.set(-4.0f, 22.0f);
      bd.angle = -0.25f;

      Body ground = getWorld().createBody(bd);
      ground.createFixture(shape, 0.0f);
    }

    {
      PolygonShape shape = new PolygonShape();
      shape.setAsBox(0.25f, 1.0f);

      BodyDef bd = new BodyDef();
      bd.position.set(10.5f, 19.0f);

      Body ground = getWorld().createBody(bd);
      ground.createFixture(shape, 0.0f);
    }

    {
      PolygonShape shape = new PolygonShape();
      shape.setAsBox(13.0f, 0.25f);

      BodyDef bd = new BodyDef();
      bd.position.set(4.0f, 14.0f);
      bd.angle = 0.25f;

      Body ground = getWorld().createBody(bd);
      ground.createFixture(shape, 0.0f);
    }

    {
      PolygonShape shape = new PolygonShape();
      shape.setAsBox(0.25f, 1.0f);

      BodyDef bd = new BodyDef();
      bd.position.set(-10.5f, 11.0f);

      Body ground = getWorld().createBody(bd);
      ground.createFixture(shape, 0.0f);
    }

    {
      PolygonShape shape = new PolygonShape();
      shape.setAsBox(13.0f, 0.25f);

      BodyDef bd = new BodyDef();
      bd.position.set(-4.0f, 6.0f);
      bd.angle = -0.25f;

      Body ground = getWorld().createBody(bd);
      ground.createFixture(shape, 0.0f);
    }

    {
      PolygonShape shape = new PolygonShape();
      shape.setAsBox(0.5f, 0.5f);

      FixtureDef fd = new FixtureDef();
      fd.shape = shape;
      fd.density = 25.0f;

      float friction[] = { 0.75f, 0.5f, 0.35f, 0.1f, 0.0f };

      for (int i = 0; i < 5; ++i) {
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(-15.0f + 4.0f * i, 28.0f);
        Body body = getWorld().createBody(bd);

        fd.friction = friction[i];
        body.createFixture(fd);
      }
    }
  }

}

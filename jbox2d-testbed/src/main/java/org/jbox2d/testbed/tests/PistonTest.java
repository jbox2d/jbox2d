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
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.PrismaticJointDef;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.framework.TestbedTest;

public class PistonTest extends TestbedTest {
  
  private boolean bullet = false;

  @Override
  public boolean isSaveLoadEnabled() {
    return true;
  }

  @Override
  public void initTest(boolean deserialized) {
    if (deserialized) {
      return;
    }
    World world = getWorld();
    Body ground = null;
    {
      BodyDef bd = new BodyDef();
      ground = getWorld().createBody(bd);

      PolygonShape shape = new PolygonShape();
      shape.setAsBox(5.0f, 100.0f);
      bd = new BodyDef();
      bd.type = BodyType.STATIC;
      FixtureDef sides = new FixtureDef();
      sides.shape = shape;
      sides.density = 0;
      sides.friction = 0;
      sides.restitution = .8f;
      sides.filter.categoryBits = 4;
      sides.filter.maskBits = 2;

      bd.position.set(-10.01f, 50.0f);
      Body bod = world.createBody(bd);
      bod.createFixture(sides);
      bd.position.set(10.01f, 50.0f);
      bod = world.createBody(bd);
      bod.createFixture(sides);
    }

    // turney
    {
      CircleShape cd;
      FixtureDef fd = new FixtureDef();
      BodyDef bd = new BodyDef();
      bd.type = BodyType.DYNAMIC;
      int numPieces = 5;
      float radius = 4f;
      bd.position = new Vec2(0.0f, 25.0f);
      Body body = getWorld().createBody(bd);
      for (int i = 0; i < numPieces; i++) {
        cd = new CircleShape();
        cd.m_radius = .5f;
        fd.shape = cd;
        fd.density = 25;
        fd.friction = .1f;
        fd.restitution = .9f;
        float xPos = radius * (float) Math.cos(2f * Math.PI * (i / (float) (numPieces)));
        float yPos = radius * (float) Math.sin(2f * Math.PI * (i / (float) (numPieces)));
        cd.m_p.set(xPos, yPos);

        body.createFixture(fd);
      }

      RevoluteJointDef rjd = new RevoluteJointDef();
      rjd.initialize(body, getGroundBody(), body.getPosition());
      rjd.motorSpeed = MathUtils.PI;
      rjd.maxMotorTorque = 1000000.0f;
      rjd.enableMotor = true;
      getWorld().createJoint(rjd);
    }


    {
      Body prevBody = ground;

      // Define crank.
      {
        PolygonShape shape = new PolygonShape();
        shape.setAsBox(0.5f, 2.0f);

        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(0.0f, 7.0f);
        Body body = getWorld().createBody(bd);
        body.createFixture(shape, 2.0f);

        RevoluteJointDef rjd = new RevoluteJointDef();
        rjd.initialize(prevBody, body, new Vec2(0.0f, 5.0f));
        rjd.motorSpeed = 1.0f * MathUtils.PI;
        rjd.maxMotorTorque = 20000;
        rjd.enableMotor = true;
        getWorld().createJoint(rjd);

        prevBody = body;
      }

      // Define follower.
      {
        PolygonShape shape = new PolygonShape();
        shape.setAsBox(0.5f, 4.0f);

        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(0.0f, 13.0f);
        Body body = getWorld().createBody(bd);
        body.createFixture(shape, 2.0f);

        RevoluteJointDef rjd = new RevoluteJointDef();
        rjd.initialize(prevBody, body, new Vec2(0.0f, 9.0f));
        rjd.enableMotor = false;
        getWorld().createJoint(rjd);

        prevBody = body;
      }

      // Define piston
      {
        PolygonShape shape = new PolygonShape();
        shape.setAsBox(7f, 2f);

        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(0.0f, 17.0f);
        Body body = getWorld().createBody(bd);
        FixtureDef piston = new FixtureDef();
        piston.shape = shape;
        piston.density = 2;
        piston.filter.categoryBits = 1;
        piston.filter.maskBits = 2;
        body.createFixture(piston);
        body.setBullet(false);
        
        RevoluteJointDef rjd = new RevoluteJointDef();
        rjd.initialize(prevBody, body, new Vec2(0.0f, 17.0f));
        getWorld().createJoint(rjd);

        PrismaticJointDef pjd = new PrismaticJointDef();
        pjd.initialize(ground, body, new Vec2(0.0f, 17.0f), new Vec2(0.0f, 1.0f));

        pjd.maxMotorForce = 1000.0f;
        pjd.enableMotor = true;

        getWorld().createJoint(pjd);
      }

      // Create a payload
      {
        PolygonShape sd = new PolygonShape();
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        FixtureDef fixture = new FixtureDef();
        Body body;
        for (int i = 0; i < 100; ++i) {
          sd.setAsBox(0.4f, 0.3f);
          bd.position.set(-1.0f, 23.0f + i);

          bd.bullet = bullet;
          body = world.createBody(bd);
          fixture.shape = sd;
          fixture.density = .1f;
          fixture.filter.categoryBits = 2;
          fixture.filter.maskBits = 1 | 4 | 2;
          body.createFixture(fixture);
        }

        CircleShape cd = new CircleShape();
        cd.m_radius = 0.36f;
        for (int i = 0; i < 100; ++i) {
          bd.position.set(1.0f, 23.0f + i);
          bd.bullet = bullet;
          fixture.shape = cd;
          fixture.density = 2f;
          fixture.filter.categoryBits = 2;
          fixture.filter.maskBits = 1 | 4 | 2;
          body = world.createBody(bd);
          body.createFixture(fixture);
        }
        
        float angle = 0.0f;
        float delta = MathUtils.PI / 3.0f;
        Vec2 vertices[] = new Vec2[6];
        for (int i = 0; i < 6; ++i) {
          vertices[i] = new Vec2(0.3f * MathUtils.cos(angle), 0.3f * MathUtils.sin(angle));
          angle += delta;
        }

        PolygonShape shape = new PolygonShape();
        shape.set(vertices, 6);

        for (int i = 0; i < 100; ++i) {
          bd.position.set(0f, 23.0f + i);
          bd.type = BodyType.DYNAMIC;
          bd.fixedRotation = true;
          bd.bullet = bullet;
          fixture.shape = shape;
          fixture.density = 1f;
          fixture.filter.categoryBits = 2;
          fixture.filter.maskBits = 1 | 4 | 2;
          body = world.createBody(bd);
          body.createFixture(fixture);
        }
      }
    }

  }

  @Override
  public String getTestName() {
    return "Piston Stress Test";
  }
}

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

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.PrismaticJointDef;
import org.jbox2d.testbed.framework.TestbedTest;

public class PrismaticStrangenessTEst extends TestbedTest{

  @Override
  public void initTest(boolean deserialized) {
    getWorld().setGravity(new Vec2());
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
    triangleBodyDef.position.set(-5.0f, 2.0f);

    // Large triangle (recycle definitions)
    vertices[0].mulLocal(2.0f);
    vertices[1].mulLocal(2.0f);
    vertices[2].mulLocal(2.0f);
    polygon.set(vertices, 3);
    triangleBodyDef.position.set(-5.0f, 6.0f);
    triangleBodyDef.fixedRotation = true; // look at me!

    Body body2 = getWorld().createBody(triangleBodyDef);
    body2.createFixture(triangleShapeDef);

    {
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(-5.0f, 10.0f);
        Body body = getWorld().createBody(bd);

        PolygonShape p = new PolygonShape();
        p.setAsBox(0.5f, 1.0f);
        body.createFixture(p, 1.0f);

        PrismaticJointDef jd = new PrismaticJointDef();
        jd.bodyA = body2;
        jd.bodyB = body;
        jd.enableLimit = true;
        jd.localAnchorA.set(0.0f, 4.0f);
        jd.localAnchorB.setZero();
        jd.localAxisA.set(0.0f, 1.0f);
        jd.lowerTranslation = -1.0f;
        jd.upperTranslation = 1.0f;

        getWorld().createJoint(jd);
    }
  }

  @Override
  public String getTestName() {
    return "Prismatic Strangeness";
  }

}

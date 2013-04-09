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
/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.box2d.org
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.ConstantVolumeJointDef;
import org.jbox2d.testbed.framework.TestbedTest;

public class BlobTest4 extends TestbedTest {

  @Override
  public float getDefaultCameraScale() {
    return 20;
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

    Body ground = null;
    {
      PolygonShape sd = new PolygonShape();
      sd.setAsBox(50.0f, 0.4f);

      BodyDef bd = new BodyDef();
      bd.position.set(0.0f, 0.0f);
      ground = getWorld().createBody(bd);
      ground.createFixture(sd, 0f);

      sd.setAsBox(0.4f, 50.0f, new Vec2(-10.0f, 0.0f), 0.0f);
      ground.createFixture(sd, 0f);
      sd.setAsBox(0.4f, 50.0f, new Vec2(10.0f, 0.0f), 0.0f);
      ground.createFixture(sd, 0f);
    }

    ConstantVolumeJointDef cvjd = new ConstantVolumeJointDef();

    float cx = 0.0f;
    float cy = 10.0f;
    float rx = 5.0f;
    float ry = 5.0f;
    int nBodies = 20;
    float bodyRadius = 0.5f;
    for (int i = 0; i < nBodies; ++i) {
      float angle = MathUtils.map(i, 0, nBodies, 0, 2 * 3.1415f);
      BodyDef bd = new BodyDef();
      // bd.isBullet = true;
      bd.fixedRotation = true;

      float x = cx + rx * (float) Math.sin(angle);
      float y = cy + ry * (float) Math.cos(angle);
      bd.position.set(new Vec2(x, y));
      bd.type = BodyType.DYNAMIC;
      Body body = getWorld().createBody(bd);

      FixtureDef fd = new FixtureDef();
      CircleShape cd = new CircleShape();
      cd.m_radius = bodyRadius;
      fd.shape = cd;
      fd.density = 1.0f;
      body.createFixture(fd);
      cvjd.addBody(body);
    }

    cvjd.frequencyHz = 10.0f;
    cvjd.dampingRatio = 1.0f;
    cvjd.collideConnected = false;
    getWorld().createJoint(cvjd);

    BodyDef bd2 = new BodyDef();
    bd2.type = BodyType.DYNAMIC;
    PolygonShape psd = new PolygonShape();
    psd.setAsBox(3.0f, 1.5f, new Vec2(cx, cy + 15.0f), 0.0f);
    bd2.position = new Vec2(cx, cy + 15.0f);
    Body fallingBox = getWorld().createBody(bd2);
    fallingBox.createFixture(psd, 1.0f);
  }

  @Override
  public String getTestName() {
    return "Blob Joint";
  }

}

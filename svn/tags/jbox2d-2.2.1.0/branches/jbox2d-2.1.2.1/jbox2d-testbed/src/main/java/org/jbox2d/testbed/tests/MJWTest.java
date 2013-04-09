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
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.testbed.framework.TestbedTest;

public class MJWTest extends TestbedTest {
   
   @Override
   public void initTest(boolean argDeserialized) {
      setTitle("Couple of Things Test");
      
      getWorld().setGravity(new Vec2());

      for (int i = 0; i < 2; i++)
      {
//         CircleShape circleShape = new CircleShape();
//         circleShape.m_radius = 1;
//         Shape shape = circleShape;
         PolygonShape polygonShape = new PolygonShape();
         polygonShape.setAsBox(1, 1);
         Shape shape = polygonShape;

         BodyDef bodyDef = new BodyDef();
         bodyDef.type = BodyType.DYNAMIC;
         bodyDef.position.set(5 * i, 0);
         bodyDef.angle = (float) (Math.PI / 4 * i);
         bodyDef.allowSleep = false;
         Body body = getWorld().createBody(bodyDef);
         body.createFixture(shape, 5.0f);
         
         body.applyForce(new Vec2(-10000 * (i - 1), 0), new Vec2());
      }
   }

   /**
    * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
    */
   @Override
   public String getTestName() {
      return "Couple of Things";
   }
}

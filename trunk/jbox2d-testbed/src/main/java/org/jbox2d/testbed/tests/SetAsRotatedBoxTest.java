package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.testbed.framework.TestbedTest;

public class SetAsRotatedBoxTest extends TestbedTest
{
   @Override
   public void initTest(boolean argDeserialized)
   {
      {
         PolygonShape shape = new PolygonShape();
         shape.setAsBox(1, 1, new Vec2(0, 0), .5f * 3.1416f);
   
         BodyDef bodyDef = new BodyDef();
         bodyDef.type = BodyType.DYNAMIC;
         bodyDef.position.set(0, 3);
         bodyDef.angle = 3;
         bodyDef.allowSleep = false;
         Body body = getWorld().createBody(bodyDef);
         body.createFixture(shape, 1);
      }
      
      {
         PolygonShape shape = new PolygonShape();
         shape.setAsBox(1, 1);
         
         BodyDef bodyDef = new BodyDef();
         bodyDef.type = BodyType.STATIC;
         Body ground = getWorld().createBody(bodyDef);
         ground.createFixture(shape, 0);
      }
   }

   @Override
   public String getTestName()
   {
      return "Set as Rotated Box";
   }
}
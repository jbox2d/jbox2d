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
   public void initTest() {
      setTitle("Couple of Things Test");
      
      world.setGravity(new Vec2());

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
         Body body = world.createBody(bodyDef);
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
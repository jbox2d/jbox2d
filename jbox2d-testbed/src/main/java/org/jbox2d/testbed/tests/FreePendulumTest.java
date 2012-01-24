
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.framework.TestbedTest;

public class FreePendulumTest extends TestbedTest
{
   private final boolean switchBodiesInJoint;

   public FreePendulumTest(boolean switchBodiesInJoint)
   {
      this.switchBodiesInJoint = switchBodiesInJoint;
   }
   
   @Override
   public void initTest(boolean argDeserialized)
   {
      Body pendulum;
      Body base;
      Body ground;

      {
         CircleShape circleShape = new CircleShape();
         circleShape.m_radius = 1;
         Shape shape = circleShape;
   
         BodyDef bodyDef = new BodyDef();
         bodyDef.type = BodyType.DYNAMIC;
         bodyDef.position.set(-5, 0);
         bodyDef.allowSleep = false;
         pendulum = getWorld().createBody(bodyDef);
         pendulum.createFixture(shape, 1);
      }
      
      {
         PolygonShape shape = new PolygonShape();
         shape.setAsBox(1, 1);
         
         BodyDef bodyDef = new BodyDef();
         bodyDef.type = BodyType.DYNAMIC;
         bodyDef.position.set(0, 2);
         bodyDef.allowSleep = false;
         base = getWorld().createBody(bodyDef);
         base.createFixture(shape, 1);
      }
      
      {
         PolygonShape shape = new PolygonShape();
         shape.setAsBox(3, 1);
         
         BodyDef bodyDef = new BodyDef();
         bodyDef.type = BodyType.STATIC;
         ground = getWorld().createBody(bodyDef);
         ground.createFixture(shape, 0);
      }
      
      RevoluteJointDef jointDef = new RevoluteJointDef();
      
      if (switchBodiesInJoint)
         jointDef.initialize(pendulum, base, new Vec2(0, 0));
      else
         jointDef.initialize(base, pendulum, new Vec2(0, 0));
      
      getWorld().createJoint(jointDef);
   }

   @Override
   public String getTestName()
   {
      return "Free Pendulum " + (switchBodiesInJoint ? "1" : "0");
   }
}
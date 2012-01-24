package org.jbox2d.testbed.tests;

import javax.swing.*;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.*;
import org.jbox2d.testbed.framework.*;
import org.jbox2d.testbed.framework.j2d.TestPanelJ2D;
import org.slf4j.*;

public class CharacterCollisionProblem extends TestbedTest {

   @Override
   public void initTest(boolean argDeserialized) {
      // character warps through this polygon
      {
         BodyDef bd = new BodyDef();
         bd.position = new Vec2(1, 0);
         Body ground = getWorld().createBody(bd);

         PolygonShape shape = new PolygonShape();
         Vec2[] vertices = new Vec2[8];
         vertices[0] = new Vec2(0.0f, -5.3f);
         vertices[1] = new Vec2(0.033333335f, -5.3333335f);
         vertices[2] = new Vec2(9.633333f, -5.3333335f);
         vertices[3] = new Vec2(9.666667f, -5.3f);
         vertices[4] = new Vec2(9.666667f, -4.7000003f);
         vertices[5] = new Vec2(9.633333f, -4.666667f);
         vertices[6] = new Vec2(0.033333335f, -4.666667f);
         vertices[7] = new Vec2(0.0f, -4.7000003f);

         shape.set(vertices, 8);
         ground.createFixture(shape, 1f);
      }

      // this character warps
      {
         BodyDef bd = new BodyDef();
         bd.position.set(4.0f, 5.0f);
         bd.type = BodyType.DYNAMIC;
         bd.fixedRotation = true;
         bd.allowSleep = false;

         Body body = getWorld().createBody(bd);

         PolygonShape shape = new PolygonShape();
         shape.setAsBox(.2f, .8f);

         FixtureDef fd = new FixtureDef();
         fd.shape = shape;
         fd.density = 20.0f;
         body.createFixture(fd);
      }

      // this one doesn't (because he doesn't land in that weird "warp spot")
      {
         BodyDef bd = new BodyDef();
         bd.position.set(6.0f, 5.0f);
         bd.type = BodyType.DYNAMIC;
         bd.fixedRotation = true;
         bd.allowSleep = false;

         Body body = getWorld().createBody(bd);

         PolygonShape shape = new PolygonShape();
         shape.setAsBox(.2f, .8f);

         FixtureDef fd = new FixtureDef();
         fd.shape = shape;
         fd.density = 20.0f;
         body.createFixture(fd);
      }
   }

   @Override
   public void step(TestbedSettings settings) {
      super.step(settings);
      addTextLine("Test that illustrates a collision problem.");
   }

   @Override
   public String getTestName() {
      return "Collision Problem";
   }

   private static final Logger log = LoggerFactory
         .getLogger(CharacterCollisionProblem.class);

   public static void main(String[] args) {
      try {
         UIManager
               .setLookAndFeel("com.sun.java.swing.plaf.nimbus.NimbusLookAndFeel");
      } catch (Exception e) {
         log.warn("Could not set the look and feel to nimbus.  "
               + "Hopefully you're on a mac so the window isn't ugly as crap.");
      }
      TestbedModel model = new TestbedModel();
      TestbedPanel panel = new TestPanelJ2D(model);
      model.addCategory("Problem");
      model.addTest(new CharacterCollisionProblem());
      JFrame testbed = new TestbedFrame(model, panel);
      testbed.setVisible(true);
      testbed.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
   }

   @Override
   public boolean isSaveLoadEnabled() {
      return true;
   }

}
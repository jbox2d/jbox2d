package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.dynamics.joints.RopeJointDef;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

public class RopeTest extends TestbedTest {

  RopeJointDef m_ropeDef;
  Joint m_rope;

  @Override
  public void initTest(boolean deserialized) {
    if (deserialized) {
      return;
    }

    Body ground = null;
    {
      BodyDef bd = new BodyDef();
      ground = getWorld().createBody(bd);

      EdgeShape shape = new EdgeShape();
      shape.set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
      ground.createFixture(shape, 0.0f);
    }

    {
      PolygonShape shape = new PolygonShape();
      shape.setAsBox(0.5f, 0.125f);

      FixtureDef fd = new FixtureDef();
      fd.shape = shape;
      fd.density = 20.0f;
      fd.friction = 0.2f;
      fd.filter.categoryBits = 0x0001;
      fd.filter.maskBits = 0xFFFF & ~0x0002;

      RevoluteJointDef jd = new RevoluteJointDef();
      jd.collideConnected = false;

      final int N = 10;
      final float y = 15.0f;
      m_ropeDef = new RopeJointDef();
      m_ropeDef.localAnchorA.set(0.0f, y);

      Body prevBody = ground;
      for (int i = 0; i < N; ++i) {
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(0.5f + 1.0f * i, y);
        if (i == N - 1) {
          shape.setAsBox(1.5f, 1.5f);
          fd.density = 100.0f;
          fd.filter.categoryBits = 0x0002;
          bd.position.set(1.0f * i, y);
          bd.angularDamping = 0.4f;
        }

        Body body = getWorld().createBody(bd);

        body.createFixture(fd);

        Vec2 anchor = new Vec2(i, y);
        jd.initialize(prevBody, body, anchor);
        getWorld().createJoint(jd);

        prevBody = body;
      }

      m_ropeDef.localAnchorB.setZero();

      float extraLength = 0.01f;
      m_ropeDef.maxLength = N - 1.0f + extraLength;
      m_ropeDef.bodyB = prevBody;
    }

    {
      m_ropeDef.bodyA = ground;
      m_rope = getWorld().createJoint(m_ropeDef);
    }
  }

  @Override
  public void keyPressed(char keyChar, int keyCode) {
    switch (keyChar) {
      case 'j':
        if (m_rope != null) {
          getWorld().destroyJoint(m_rope);
          m_rope = null;
        } else {
          m_rope = getWorld().createJoint(m_ropeDef);
        }
        break;
    }
  }

  @Override
  public synchronized void step(TestbedSettings settings) {
    super.step(settings);
    addTextLine("Press (j) to toggle the rope joint.");
    m_textLine += 15;
    if (m_rope != null) {
      addTextLine("Rope ON");
    } else {
      addTextLine("Rope OFF");
    }
  }

  @Override
  public String getTestName() {
    return "Rope Joint";
  }
}

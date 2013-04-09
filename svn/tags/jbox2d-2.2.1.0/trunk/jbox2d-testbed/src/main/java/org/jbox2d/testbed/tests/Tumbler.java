package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

public class Tumbler extends TestbedTest {
  private static final int MAX_NUM = 800;
  RevoluteJoint m_joint;
  int m_count;

  @Override
  public void initTest(boolean deserialized) {
    {
      BodyDef bd = new BodyDef();
      bd.type = BodyType.DYNAMIC;
      bd.allowSleep = false;
      bd.position.set(0.0f, 10.0f);
      Body body = m_world.createBody(bd);

      PolygonShape shape = new PolygonShape();
      shape.setAsBox(0.5f, 10.0f, new Vec2(10.0f, 0.0f), 0.0f);
      body.createFixture(shape, 5.0f);
      shape.setAsBox(0.5f, 10.0f, new Vec2(-10.0f, 0.0f), 0.0f);
      body.createFixture(shape, 5.0f);
      shape.setAsBox(10.0f, 0.5f, new Vec2(0.0f, 10.0f), 0.0f);
      body.createFixture(shape, 5.0f);
      shape.setAsBox(10.0f, 0.5f, new Vec2(0.0f, -10.0f), 0.0f);
      body.createFixture(shape, 5.0f);

      RevoluteJointDef jd = new RevoluteJointDef();
      jd.bodyA = getGroundBody();
      jd.bodyB = body;
      jd.localAnchorA.set(0.0f, 10.0f);
      jd.localAnchorB.set(0.0f, 0.0f);
      jd.referenceAngle = 0.0f;
      jd.motorSpeed = 0.05f * MathUtils.PI;
      jd.maxMotorTorque = 1e8f;
      jd.enableMotor = true;
      m_joint = (RevoluteJoint) m_world.createJoint(jd);
    }
    m_count = 0;
  }

  @Override
  public synchronized void step(TestbedSettings settings) {
    super.step(settings);

    if (m_count < MAX_NUM) {
      BodyDef bd = new BodyDef();
      bd.type = BodyType.DYNAMIC;
      bd.position.set(0.0f, 10.0f);
      Body body = m_world.createBody(bd);

      PolygonShape shape = new PolygonShape();
      shape.setAsBox(0.125f, 0.125f);
      body.createFixture(shape, 1.0f);

      ++m_count;
    }
  }

  @Override
  public String getTestName() {
    return "Tumbler";
  }
}

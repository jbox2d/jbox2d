package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.MotorJoint;
import org.jbox2d.dynamics.joints.MotorJointDef;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

public class MotorTest extends TestbedTest {
  MotorJoint m_joint;
  float m_time;
  boolean m_go;

  @Override
  public void initTest(boolean deserialized) {
    {
      EdgeShape shape = new EdgeShape();
      shape.set(new Vec2(-20.0f, 0.0f), new Vec2(20.0f, 0.0f));
      FixtureDef fd = new FixtureDef();
      fd.shape = shape;
      getGroundBody().createFixture(fd);
    }

    // Define motorized body
    {
      BodyDef bd = new BodyDef();
      bd.type = BodyType.DYNAMIC;
      bd.position.set(0.0f, 8.0f);
      Body body = getWorld().createBody(bd);

      PolygonShape shape = new PolygonShape();
      shape.setAsBox(2.0f, 0.5f);

      FixtureDef fd = new FixtureDef();
      fd.shape = shape;
      fd.friction = 0.6f;
      fd.density = 2.0f;
      body.createFixture(fd);

      MotorJointDef mjd = new MotorJointDef();
      mjd.initialize(getGroundBody(), body);
      mjd.maxForce = 1000.0f;
      mjd.maxTorque = 1000.0f;
      m_joint = (MotorJoint) m_world.createJoint(mjd);
    }

    m_go = false;
    m_time = 0.0f;
  }

  @Override
  public void keyPressed(char keyCar, int keyCode) {
    super.keyPressed(keyCar, keyCode);

    switch (keyCar) {
      case 's':
        m_go = !m_go;
    }
  }

  // pooling
  Vec2 linearOffset = new Vec2();
  Color3f color = new Color3f(0.9f, 0.9f, 0.9f);

  @Override
  public void step(TestbedSettings settings) {
    float hz = settings.getSetting(TestbedSettings.Hz).value;
    if (m_go && hz > 0.0f) {
      m_time += 1.0f / hz;
    }

    linearOffset.x = 6.0f * MathUtils.sin(2.0f * m_time);
    linearOffset.y = 8.0f + 4.0f * MathUtils.sin(1.0f * m_time);

    float angularOffset = 4.0f * m_time;

    m_joint.setLinearOffset(linearOffset);
    m_joint.setAngularOffset(angularOffset);

    getDebugDraw().drawPoint(linearOffset, 4.0f, color);
    super.step(settings);
    addTextLine("Keys: (s) pause");
  }

  @Override
  public String getTestName() {
    return "Motor Joint";
  }
}

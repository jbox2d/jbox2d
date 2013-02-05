package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.WheelJoint;
import org.jbox2d.dynamics.joints.WheelJointDef;
import org.jbox2d.testbed.framework.TestbedTest;

public class WheelTest extends TestbedTest {
  
  private final Vec2  gravity = new Vec2(0.0f, -10.0f);
  private Fixture     wheel_fixture;
  private Fixture     floor_fixture;
  private Fixture     roof_fixture;
  private Fixture     left_fixture;
  private Fixture     right_fixture;
  private Fixture     box_fixture;
  private WheelJoint   joint;
  
  @Override
  public void initTest(boolean deserialized) {
    if (deserialized) {
      return;
    }
    m_world = new World(this.gravity);

    /*
     * Ground.
     */

    {
      final BodyDef bd = new BodyDef();
      bd.position.x = 20.0f;
      bd.position.y = 2.0f;
      bd.type = BodyType.STATIC;
      final Body b = this.m_world.createBody(bd);
      final PolygonShape s = new PolygonShape();

      s.setAsBox(20f, 0.5f, new Vec2(0.0f, 0.0f), 0.0f);
      final FixtureDef floor = new FixtureDef();
      floor.density = 1.0f;
      floor.friction = 0.5f;
      floor.shape = s;
      this.floor_fixture = b.createFixture(floor);

      s.setAsBox(20f, 0.5f, new Vec2(0.0f, 17.0f), 0.0f);
      final FixtureDef roof = new FixtureDef();
      roof.density = 1.0f;
      roof.friction = 0.5f;
      roof.shape = s;
      this.roof_fixture = b.createFixture(roof);

      s.setAsBox(0.5f, 20.0f, new Vec2(-18.0f, 0.0f), 0.0f);
      final FixtureDef left = new FixtureDef();
      left.density = 1.0f;
      left.friction = 0.5f;
      left.shape = s;
      this.left_fixture = b.createFixture(left);

      s.setAsBox(0.5f, 20.0f, new Vec2(18.0f, 0.0f), 0.0f);
      final FixtureDef right = new FixtureDef();
      right.density = 1.0f;
      right.friction = 0.5f;
      right.shape = s;
      this.right_fixture = b.createFixture(right);
    }

    /*
     * Wheel.
     */

    {
      final CircleShape wheel_shape = new CircleShape();
      wheel_shape.m_radius = 2.0f;

      final FixtureDef wheel_fixture_def = new FixtureDef();
      wheel_fixture_def.density = 1.0f;
      wheel_fixture_def.friction = 0.5f;
      wheel_fixture_def.shape = wheel_shape;

      final BodyDef wheel_body = new BodyDef();
      wheel_body.position.x = 20.0f;
      wheel_body.position.y = 10.0f;
      wheel_body.type = BodyType.DYNAMIC;

      final Body b = this.m_world.createBody(wheel_body);
      this.wheel_fixture = b.createFixture(wheel_fixture_def);
    }

    /*
     * Box above wheel.
     */

    {
      final PolygonShape box_shape = new PolygonShape();
      box_shape.setAsBox(1f, 1f, new Vec2(0.0f, 0.0f), 0.0f);

      final BodyDef box_body = new BodyDef();
      box_body.position.x = 20.0f;
      box_body.position.y = 15.0f;
      box_body.type = BodyType.DYNAMIC;
      box_body.fixedRotation = true;

      final FixtureDef box_fixture_def = new FixtureDef();
      box_fixture_def.density = 1.0f;
      box_fixture_def.friction = 0.5f;
      box_fixture_def.shape = box_shape;

      final Body b = this.m_world.createBody(box_body);
      this.box_fixture = b.createFixture(box_fixture_def);
    }

    /*
     * Joint.
     */

    {
      final WheelJointDef jd = new WheelJointDef();
      jd.initialize(
        this.box_fixture.getBody(),
        this.wheel_fixture.getBody(),
        this.wheel_fixture.getBody().getWorldCenter(),
        new Vec2(0.0f, 1.0f));
      jd.motorSpeed = 100.0f;
      jd.maxMotorTorque = 1000.0f;
      jd.enableMotor = true;

      this.joint = (WheelJoint) this.m_world.createJoint(jd);
      assert this.joint != null;
    }

  }

  @Override
  public String getTestName() {
    // TODO Auto-generated method stub
    return null;
  }

}

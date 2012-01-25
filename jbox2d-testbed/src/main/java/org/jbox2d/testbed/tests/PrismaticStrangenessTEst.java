package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.PrismaticJointDef;
import org.jbox2d.testbed.framework.TestbedTest;

public class PrismaticStrangenessTEst extends TestbedTest{

  @Override
  public void initTest(boolean deserialized) {
    getWorld().setGravity(new Vec2());
    Vec2 vertices[] = new Vec2[3];
    vertices[0] = new Vec2(-1.0f, 0.0f);
    vertices[1] = new Vec2(1.0f, 0.0f);
    vertices[2] = new Vec2(0.0f, 2.0f);
    PolygonShape polygon = new PolygonShape();
    polygon.set(vertices, 3);

    FixtureDef triangleShapeDef = new FixtureDef();
    triangleShapeDef.shape = polygon;
    triangleShapeDef.density = 1.0f;

    BodyDef triangleBodyDef = new BodyDef();
    triangleBodyDef.type = BodyType.DYNAMIC;
    triangleBodyDef.position.set(-5.0f, 2.0f);

    // Large triangle (recycle definitions)
    vertices[0].mulLocal(2.0f);
    vertices[1].mulLocal(2.0f);
    vertices[2].mulLocal(2.0f);
    polygon.set(vertices, 3);
    triangleBodyDef.position.set(-5.0f, 6.0f);
    triangleBodyDef.fixedRotation = true; // look at me!

    Body body2 = getWorld().createBody(triangleBodyDef);
    body2.createFixture(triangleShapeDef);

    {
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(-5.0f, 10.0f);
        Body body = getWorld().createBody(bd);

        PolygonShape p = new PolygonShape();
        p.setAsBox(0.5f, 1.0f);
        body.createFixture(p, 1.0f);

        PrismaticJointDef jd = new PrismaticJointDef();
        jd.bodyA = body2;
        jd.bodyB = body;
        jd.enableLimit = true;
        jd.localAnchorA.set(0.0f, 4.0f);
        jd.localAnchorB.setZero();
        jd.localAxisA.set(0.0f, 1.0f);
        jd.lowerTranslation = -1.0f;
        jd.upperTranslation = 1.0f;

        getWorld().createJoint(jd);
    }
  }

  @Override
  public String getTestName() {
    return "Prismatic Strangeness";
  }

}

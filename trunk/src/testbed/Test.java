package testbed;

import java.awt.Color;
import java.util.Random;

import collision.AABB;
import collision.BroadPhase;
import collision.Manifold;
import collision.Pair;
import collision.Proxy;
import collision.Shape;
import collision.ShapeDescription;
import collision.ShapeType;

import common.Settings;
import common.Vec2;

import dynamics.Body;
import dynamics.BodyDescription;
import dynamics.World;
import dynamics.contacts.Contact;
import dynamics.joints.Joint;
import dynamics.joints.MouseDescription;
import dynamics.joints.MouseJoint;

public class Test {
    protected int m_textLine;

    protected World m_world;

    protected Body m_bomb;

    protected MouseJoint m_mouseJoint;

    public Test() {
        m_world = new World(new AABB(new Vec2(-100, -100), new Vec2(100, 100)),
                new Vec2(0.0f, 10.0f), true);

        m_bomb = null;
        m_textLine = 30;
        m_mouseJoint = null;
    }

    void MouseDown(Vec2 p) {
        assert m_mouseJoint == null;

        // Make a small box.

        Vec2 d = new Vec2(0.001f, 0.001f);
        AABB aabb = new AABB(p.sub(d), p.add(d));

        // Query the world for overlapping shapes.
        int k_maxCount = 10;
        Shape shapes[] = m_world.Query(aabb, k_maxCount);
        Body body = null;
        for (int j = 0; j < shapes.length; j++) {
            Shape shape = shapes[j];
            if (!shape.m_body.IsStatic()) {
                boolean inside = shape.TestPoint(p);
                if (inside) {
                    body = shape.m_body;
                    break;
                }
            }
        }

        if (body != null) {
            MouseDescription md = new MouseDescription();
            md.body1 = m_world.m_groundBody;
            md.body2 = body;
            md.target = p;
            md.motorForce = 400.0f * body.m_mass;
            m_mouseJoint = (MouseJoint) m_world.CreateJoint(md);
            body.wakeUp();
        }
    }

    void MouseUp() {
        if (m_mouseJoint != null) {
            m_world.DestroyJoint(m_mouseJoint);
            m_mouseJoint = null;
        }
    }

    void MouseMove(Vec2 p) {
        if (m_mouseJoint != null) {
            m_mouseJoint.SetTarget(p);
        }
    }

    void LaunchBomb() {
        if (m_bomb == null) {
            ShapeDescription sd = new ShapeDescription();
            float a = 0.5f;
            sd.type = ShapeType.BOX_SHAPE;
            sd.box.m_extents = new Vec2(a, a);
            sd.density = 20.0f;

            BodyDescription bd = new BodyDescription();
            bd.addShape(sd);
            m_bomb = m_world.CreateBody(bd);
        }

        Random r = new Random();

        Vec2 position = new Vec2(r.nextFloat() * 30 - 15, 30.0f);
        float rotation = r.nextFloat() * 3 - 1.5f;
        m_bomb.SetRootPosition(position, rotation);
        m_bomb.m_linearVelocity = position.mul(-1.0f);
        m_bomb.m_angularVelocity = r.nextFloat() * 40 - 20;
        m_bomb.wakeUp();
    }

    void Step(TestSettings settings) {
        float timeStep = settings.hz > 0.0f ? 1.0f / settings.hz : 0.0f;

        World.s_enableWarmStarting = settings.enableWarmStarting;
        World.s_enablePositionCorrection = settings.enablePositionCorrection;

        m_world.Step(timeStep, settings.iterationCount);

        m_world.m_broadPhase.Validate();

        for (Body b = m_world.m_bodyList; b != null; b = b.m_next) {
            for (Shape s = b.m_shapeList; s != null; s = s.m_next) {
                if (b.m_invMass == 0.0f) {
                    DrawShape(s, new Color(0.5f, 0.9f, 0.5f));
                }
                else if (b.m_isSleeping) {
                    DrawShape(s, new Color(0.5f, 0.5f, 0.9f));
                }
                else if (b == m_bomb) {
                    DrawShape(s, new Color(0.9f, 0.9f, 0.4f));
                }
                else {
                    DrawShape(s, new Color(0.9f, 0.9f, 0.9f));
                }
            }
        }

        for (Joint j = m_world.m_jointList; j != null; j = j.m_next) {
            if (j != m_mouseJoint) {
                DrawJoint(j);
            }
        }

        if (settings.drawContacts) {
            for (Contact c = m_world.m_contactList; c != null; c = c.m_next) {
                for (Manifold m : c.GetManifolds()) {
                    glPointSize(4.0f);
                    glColor3f(1.0f, 0.0f, 0.0f);
                    glBegin(GL_POINTS);
                    for (int j = 0; j < m.pointCount; ++j) {
                        b2Vec2 v = m.points[j].position;
                        glVertex2f(v.x, v.y);
                    }
                    glEnd();
                    glPointSize(1.0f);
                }
            }
        }

        if (settings.drawImpulses) {
            for (Contact c = m_world.m_contactList; c != null; c = c.m_next) {
                for (Manifold m : c.GetManifolds()) {
                    glColor3f(0.9f, 0.9f, 0.3f);
                    glBegin(GL_LINES);
                    for (int32 j = 0; j < m.pointCount; ++j) {
                        b2Vec2 v1 = m.points[j].position;
                        b2Vec2 v2 = v1 + m.points[j].normalImpulse * m.normal;
                        glVertex2f(v1.x, v1.y);
                        glVertex2f(v2.x, v2.y);
                    }
                    glEnd();
                }
            }
        }

        if (settings.drawPairs) {
            BroadPhase bp = m_world.m_broadPhase;
            Vec2 invQ = new Vec2(1.0f / bp.m_quantizationFactor.x,
                    1.0f / bp.m_quantizationFactor.y);
            glColor3f(0.9f, 0.9f, 0.3f);
            glBegin(GL_LINES);
            for (int i = 0; i < bp.m_pairManager.m_pairCount; ++i) {
                Pair pair = bp.m_pairManager.m_pairs[i];
                int id1 = pair.proxyId1;
                int id2 = pair.proxyId2;
                Proxy p1 = bp.m_proxyPool[id1];
                Proxy p2 = bp.m_proxyPool[id2];

                AABB b1 = new AABB();
                AABB b2 = new AABB();
                b1.minVertex.x = bp.m_worldAABB.minVertex.x + invQ.x
                        * bp.m_bounds[0][p1.lowerBounds[0]].value;
                b1.minVertex.y = bp.m_worldAABB.minVertex.y + invQ.y
                        * bp.m_bounds[1][p1.lowerBounds[1]].value;
                b1.maxVertex.x = bp.m_worldAABB.minVertex.x + invQ.x
                        * bp.m_bounds[0][p1.upperBounds[0]].value;
                b1.maxVertex.y = bp.m_worldAABB.minVertex.y + invQ.y
                        * bp.m_bounds[1][p1.upperBounds[1]].value;
                b2.minVertex.x = bp.m_worldAABB.minVertex.x + invQ.x
                        * bp.m_bounds[0][p2.lowerBounds[0]].value;
                b2.minVertex.y = bp.m_worldAABB.minVertex.y + invQ.y
                        * bp.m_bounds[1][p2.lowerBounds[1]].value;
                b2.maxVertex.x = bp.m_worldAABB.minVertex.x + invQ.x
                        * bp.m_bounds[0][p2.upperBounds[0]].value;
                b2.maxVertex.y = bp.m_worldAABB.minVertex.y + invQ.y
                        * bp.m_bounds[1][p2.upperBounds[1]].value;

                Vec2 x1 = b1.minVertex.add(b1.maxVertex).mul(0.5f);
                Vec2 x2 = b2.minVertex.add(b2.maxVertex).mul(0.5f);

                glVertex2f(x1.x, x1.y);
                glVertex2f(x2.x, x2.y);
            }
            glEnd();
        }

        if (settings.drawAABBs) {
            BroadPhase bp = m_world.m_broadPhase;
            Vec2 invQ = new Vec2(1.0f / bp.m_quantizationFactor.x,
                    1.0f / bp.m_quantizationFactor.y);
            glColor3f(0.9f, 0.3f, 0.9f);
            for (int i = 0; i < Settings.maxProxies; ++i) {
                Proxy p = bp.m_proxyPool[i];
                if (p.IsValid() == false) {
                    continue;
                }

                AABB b = new AABB();
                b.minVertex.x = bp.m_worldAABB.minVertex.x + invQ.x
                        * bp.m_bounds[0][p.lowerBounds[0]].value;
                b.minVertex.y = bp.m_worldAABB.minVertex.y + invQ.y
                        * bp.m_bounds[1][p.lowerBounds[1]].value;
                b.maxVertex.x = bp.m_worldAABB.minVertex.x + invQ.x
                        * bp.m_bounds[0][p.upperBounds[0]].value;
                b.maxVertex.y = bp.m_worldAABB.minVertex.y + invQ.y
                        * bp.m_bounds[1][p.upperBounds[1]].value;

                glBegin(GL_LINE_LOOP);
                glVertex2f(b.minVertex.x, b.minVertex.y);
                glVertex2f(b.maxVertex.x, b.minVertex.y);
                glVertex2f(b.maxVertex.x, b.maxVertex.y);
                glVertex2f(b.minVertex.x, b.maxVertex.y);
                glEnd();
            }
        }

        if (settings.drawStats) {
            DrawString(5, m_textLine,
                    "proxies(max) = %d(%d), pairs(max) = %d(%d)",
                    m_world.m_broadPhase.m_proxyCount, Settings.maxProxies,
                    m_world.m_broadPhase.m_pairManager.m_pairCount,
                    Settings.maxPairs);

            m_textLine += 15;

            DrawString(5, m_textLine, "bodies/contacts/joints = %d/%d/%d",
                    m_world.m_bodyCount, m_world.m_contactCount,
                    m_world.m_jointCount);
        }

        if (m_mouseJoint != null) {
            Body body = m_mouseJoint.m_body2;
            Vec2 p1 = body.m_position.add(body.m_R
                    .mul(m_mouseJoint.m_localAnchor));
            Vec2 p2 = m_mouseJoint.m_target;

            glPointSize(4.0f);
            glColor3f(0.0f, 1.0f, 0.0f);
            glBegin(GL_POINTS);
            glVertex2f(p1.x, p1.y);
            glVertex2f(p2.x, p2.y);
            glEnd();
            glPointSize(1.0f);

            glColor3f(0.8f, 0.8f, 0.8f);
            glBegin(GL_LINES);
            glVertex2f(p1.x, p1.y);
            glVertex2f(p2.x, p2.y);
            glEnd();
        }
    }
}

package dynamics;

import collision.MassData;
import collision.Shape;
import collision.ShapeDescription;

import common.Mat22;
import common.Settings;
import common.Vec2;

import dynamics.contacts.ContactNode;
import dynamics.joints.JointNode;

public class Body {
    public Vec2 m_position; // center of mass position

    public float m_rotation;

    public Mat22 m_R;

    public Vec2 m_linearVelocity;

    public float m_angularVelocity;

    public Vec2 m_force;

    public float m_torque;

    public Vec2 m_center; // local vector from client origin to center of mass

    public World m_world;

    public Body m_prev;

    public Body m_next;

    public Shape m_shapeList;

    public JointNode m_jointList;

    public ContactNode m_contactList;

    public float m_mass, m_invMass;

    public float m_I, m_invI;

    public float m_sleepTime;

    public boolean m_allowSleep;

    public boolean m_isSleeping;

    public boolean m_islandFlag;

    public boolean IsStatic() {
        return m_invMass == 0.0f;
    }

    // Get the position of the body's origin. The body's origin does not
    // necessarily coincide with the center of mass. It depends on how the
    // shapes are created.
    public Vec2 GetOriginPosition() {
        return (m_position.sub(m_R.mul(m_center)));
    }

    // Get the rotation in radians.
    public float GetRotation() {
        return m_rotation;
    }

    Body(BodyDescription bd, World world) {
        m_position = bd.position.clone();
        m_rotation = bd.rotation;
        m_R = new Mat22(m_rotation);
        m_linearVelocity = bd.linearVelocity.clone();
        m_angularVelocity = bd.angularVelocity;
        m_world = world;

        m_force = new Vec2(0.0f, 0.0f);
        m_torque = 0.0f;

        m_mass = 0.0f;

        MassData massDatas[] = new MassData[Settings.maxShapesPerBody];

        // Compute the shape mass properties, the bodies total mass and COM.
        m_center = new Vec2(0.0f, 0.0f);

        for (int i = 0; i < Settings.maxShapesPerBody; ++i) {
            ShapeDescription sd = bd.shapes[i];
            if (sd == null) {
                break;
            }
            massDatas[i] = new MassData();
            MassData massData = massDatas[i];
            sd.computeMass(massData);
            m_mass += massData.mass;
            m_center.addLocal(sd.localPosition.clone()
                    .addLocal(massData.center).mulLocal(massData.mass));
        }

        // Compute center of mass, and shift the origin to the COM.
        if (m_mass > 0.0f) {
            m_center.mulLocal(1.0f / m_mass);
            m_position.addLocal(m_R.mul(m_center));
        }

        // Compute the moment of inertia.
        m_I = 0.0f;
        for (int i = 0; i < Settings.maxShapesPerBody; ++i) {
            ShapeDescription sd = bd.shapes[i];
            if (sd == null) {
                break;
            }
            MassData massData = massDatas[i];
            m_I += massData.I;
            Vec2 r = sd.localPosition.clone().addLocal(massData.center)
                    .subLocal(m_center);
            m_I += massData.mass * Vec2.dot(r, r);
        }

        if (m_mass > 0.0f) {
            m_invMass = 1.0f / m_mass;
        }
        else {
            m_invMass = 0.0f;
        }

        if (m_I > 0.0f) {
            m_invI = 1.0f / m_I;
        }
        else {
            m_invI = 0.0f;
        }

        m_jointList = null;
        m_contactList = null;
        m_prev = null;
        m_next = null;

        // Create the shapes.
        m_shapeList = null;
        for (int i = 0; i < Settings.maxShapesPerBody; ++i) {
            ShapeDescription sd = bd.shapes[i];
            if (sd == null) {
                break;
            }
            Shape shape = Shape.Create(sd, this, m_center, massDatas[i]);
            // System.out.println(sd.type);
            // if (shape == null) System.out.println(shape);
            shape.m_next = m_shapeList;
            m_shapeList = shape;
        }

        m_sleepTime = 0.0f;
        m_allowSleep = bd.allowSleep;
        m_isSleeping = bd.isSleeping;
        if (m_isSleeping == true || m_invMass == 0.0f) {
            m_linearVelocity.set(0.0f, 0.0f);
            m_angularVelocity = 0.0f;
        }
    }

    public void SetRootPosition(Vec2 position, float rotation) {
        m_rotation = rotation;
        m_R.set(m_rotation);
        m_position = position.add(m_R.mul(m_center));

        for (Shape s = m_shapeList; s != null; s = s.m_next) {
            s.m_position = m_position.add(m_R.mul(s.m_localPosition));
            s.m_rotation = m_rotation + s.m_localRotation;
            s.m_R.set(s.m_rotation);
            s.UpdateProxy();
        }

        m_world.m_broadPhase.Flush();
    }

    void SynchronizeShapes() {
        for (Shape s = m_shapeList; s != null; s = s.m_next) {
            s.m_position = m_position.add(m_R.mul(s.m_localPosition));
            s.m_rotation = m_rotation + s.m_localRotation;
            s.m_R.set(s.m_rotation);
            s.UpdateProxy();
        }
    }

    boolean isSleeping() {
        return m_isSleeping;
    }

    public void wakeUp() {
        m_isSleeping = false;
        m_sleepTime = 0.0f;
    }

    public boolean isConnected(Body other) {
        for (JointNode jn = m_jointList; jn != null; jn = jn.next) {
            if (jn.other == other)
                return true;
        }

        return false;
    }

}

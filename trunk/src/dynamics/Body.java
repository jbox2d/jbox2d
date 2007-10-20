package dynamics;

import collision.MassData;
import collision.Shape;
import collision.ShapeDef;

import common.Mat22;
import common.Settings;
import common.Vec2;

import dynamics.contacts.ContactNode;
import dynamics.joints.JointNode;

public class Body {
    public static final int e_staticFlag = 0x0001;

    public static final int e_frozenFlag = 0x0002;

    public static final int e_islandFlag = 0x0004;

    public static final int e_sleepFlag = 0x0008;

    public static final int e_allowSleepFlag = 0x0010;

    public static final int e_destroyFlag = 0x0020;

    public int m_flags;

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

    public int m_shapeCount;

    public JointNode m_jointList;

    public ContactNode m_contactList;

    public float m_mass, m_invMass;

    public float m_I, m_invI;

    public float m_sleepTime;

    public Object m_userData;

    public Body(BodyDef bd, World world) {
        m_flags = 0;
        m_position = bd.position.clone();
        m_rotation = bd.rotation;
        m_R = new Mat22(m_rotation);
        // m_linearVelocity = bd.linearVelocity.clone();
        // m_angularVelocity = bd.angularVelocity;
        m_world = world;

        m_force = new Vec2(0.0f, 0.0f);
        m_torque = 0.0f;

        m_mass = 0.0f;

        MassData massDatas[] = new MassData[Settings.maxShapesPerBody];

        // Compute the shape mass properties, the bodies total mass and COM.
        m_shapeCount = 0;
        m_center = new Vec2(0.0f, 0.0f);

        for (int i = 0; i < Settings.maxShapesPerBody; ++i) {
            ShapeDef sd = bd.shapes[i];
            if (sd == null) {
                break;
            }
            massDatas[i] = new MassData();
            MassData massData = massDatas[i];
            sd.computeMass(massData);
            m_mass += massData.mass;
            m_center.addLocal(sd.localPosition.clone()
                    .addLocal(massData.center).mulLocal(massData.mass));
            ++m_shapeCount;
        }

        // Compute center of mass, and shift the origin to the COM.
        if (m_mass > 0.0f) {
            m_center.mulLocal(1.0f / m_mass);
            m_position.addLocal(m_R.mul(m_center));
        }
        else {
            m_flags |= e_staticFlag;
        }

        // Compute the moment of inertia.
        m_I = 0.0f;
        for (int i = 0; i < m_shapeCount; ++i) {
            ShapeDef sd = bd.shapes[i];
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

        if (m_I > 0.0f && bd.preventRotation == false) {
            m_invI = 1.0f / m_I;
        }
        else {
            m_I = 0.0f;
            m_invI = 0.0f;
        }

        // Compute the center of mass velocity.
        m_linearVelocity = bd.linearVelocity.add(Vec2.cross(bd.angularVelocity,
                m_center));
        m_angularVelocity = bd.angularVelocity;

        m_jointList = null;
        m_contactList = null;
        m_prev = null;
        m_next = null;

        // Create the shapes.
        m_shapeList = null;
        for (int i = 0; i < m_shapeCount; ++i) {
            ShapeDef sd = bd.shapes[i];
            Shape shape = Shape.create(sd, this, m_center);
            shape.m_next = m_shapeList;
            m_shapeList = shape;
        }

        m_sleepTime = 0.0f;
        if (bd.allowSleep) {
            m_flags |= e_allowSleepFlag;
        }
        if (bd.isSleeping) {
            m_flags |= e_sleepFlag;
        }

        if (((m_flags & e_sleepFlag) > 0) || m_invMass == 0.0f) {
            m_linearVelocity.set(0.0f, 0.0f);
            m_angularVelocity = 0.0f;
        }

        m_userData = bd.userData;
    }

    // Get the position of the body's origin. The body's origin does not
    // necessarily coincide with the center of mass. It depends on how the
    // shapes are created.
    public Vec2 getOriginPosition() {
        return (m_position.sub(m_R.mul(m_center)));
    }

    public Vec2 getCenterPosition() {
        return m_position;
    }

    // Get the rotation in radians.
    public float getRotation() {
        return m_rotation;
    }

    public Mat22 getRotationMatrix() {
        return m_R;
    }

    public void setLinearVelocity(Vec2 v) {
        m_linearVelocity.set(v);
    }

    public Vec2 getLinearVelocity() {
        return m_linearVelocity;
    }

    public void setAngularVelocity(float w) {
        m_angularVelocity = w;
    }

    public float getAngularVelocity() {
        return m_angularVelocity;
    }

    public void applyForce(Vec2 force, Vec2 point) {
        if (isSleeping() == false) {
            m_force.x += force.x;
            m_force.y += force.y;
            m_torque += Vec2.cross(point.sub(m_position), force);
        }
    }

    public void applyTorque(float torque) {
        if (isSleeping() == false) {
            m_torque += torque;
        }
    }

    public void applyImpulse(Vec2 impulse, Vec2 point) {
        if (isSleeping() == false) {
            m_linearVelocity.x += m_invMass * impulse.x;
            m_linearVelocity.y += m_invMass * impulse.y;
            m_angularVelocity += m_invI
                    * Vec2.cross(point.sub(m_position), impulse);
        }
    }

    public float getMass() {
        return m_mass;
    }

    public float getInertia() {
        return m_I;
    }

    public Vec2 getWorldPoint(Vec2 localPoint) {
        return m_position.add(m_R.mul(localPoint));
    }

    public Vec2 getWorldVector(Vec2 localVector) {
        return m_R.mul(localVector);
    }

    public Vec2 getLocalPoint(Vec2 worldPoint) {
        return m_R.mulT(worldPoint.sub(m_position));
    }

    public Vec2 getLocalVector(Vec2 worldVector) {
        return m_R.mulT(worldVector);
    }

    public boolean isStatic() {
        return (m_flags & e_staticFlag) == e_staticFlag;
    }

    public boolean isFrozen() {
        return (m_flags & e_frozenFlag) == e_frozenFlag;
    }

    public boolean isSleeping() {
        return (m_flags & e_sleepFlag) == e_sleepFlag;
    }

    public void allowSleeping(boolean flag) {
        if (flag) {
            m_flags |= e_allowSleepFlag;
        }
        else {
            m_flags &= ~e_allowSleepFlag;
            wakeUp();
        }
    }

    public Shape getShapeList() {
        return m_shapeList;
    }

    public ContactNode getContactList() {
        return m_contactList;
    }

    public JointNode getJointList() {
        return m_jointList;
    }

    public Body getNext() {
        return m_next;
    }

    public Object getUserData() {
        return m_userData;
    }

    public void destructor() {
        Shape s = m_shapeList;
        while (s != null) {
            Shape s0 = s;
            s = s.m_next;
            s0.destructor();
        }
    }

    public void setOriginPosition(Vec2 position, float rotation) {
        if (isFrozen()) {
            return;
        }

        m_rotation = rotation;
        m_R.setAngle(m_rotation);
        m_position = position.add(m_R.mul(m_center));

        for (Shape s = m_shapeList; s != null; s = s.m_next) {
            s.synchronize(m_position, m_R);
        }

        m_world.m_broadPhase.flush();
    }

    public void setCenterPosition(Vec2 position, float rotation) {
        if (isFrozen()) {
            return;
        }

        m_rotation = rotation;
        m_R.setAngle(m_rotation);
        m_position = position.clone();

        for (Shape s = m_shapeList; s != null; s = s.m_next) {
            s.synchronize(m_position, m_R);
        }

        m_world.m_broadPhase.flush();
    }

    void synchronizeShapes() {
        for (Shape s = m_shapeList; s != null; s = s.m_next) {
            s.synchronize(m_position, m_R);
        }
    }

    public void freeze() {
        m_flags |= e_frozenFlag;
        m_linearVelocity.setZero();
        m_angularVelocity = 0.0f;
    }

    public void wakeUp() {
        m_flags &= ~e_sleepFlag;
        m_sleepTime = 0.0f;
    }

    public boolean isConnected(Body other) {
        for (JointNode jn = m_jointList; jn != null; jn = jn.next) {
            if (jn.other == other)
                return jn.joint.m_collideConnected == false;
        }

        return false;
    }
}
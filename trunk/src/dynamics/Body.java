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
    

    // Get the position of the body's origin. The body's origin does not
    // necessarily coincide with the center of mass. It depends on how the
    // shapes are created.
    public Vec2 GetOriginPosition() {
        return (m_position.sub(m_R.mul(m_center)));
    }
    
    public Vec2 GetCenterPosition() {
        return m_position;
    }

    // Get the rotation in radians.
    public float GetRotation() {
        return m_rotation;
    }

    public Mat22 GetRotationMatrix() {
        return m_R;
    }
    
    public void SetLinearVelocity(Vec2 v) {
        m_linearVelocity.set(v);
    }
    
    public Vec2 GetLinearVelocity() {
        return m_linearVelocity;
    }
    
    public void SetAngularVelocity(float w) {
        m_angularVelocity = w;
    }
    
    public float GetAngularVelocity() {
        return m_angularVelocity;
    }
    
    public void ApplyForce(Vec2 force, Vec2 point) {
        if (isSleeping() == false) {
            m_force.x += force.x;
            m_force.y += force.y;
            m_torque += Vec2.cross(point.sub(m_position), force);
        }
    }
    
    public void ApplyTorque(float torque) {
        if (isSleeping() == false) {
            m_torque += torque;
        }
    }
    
    public void ApplyImpulse(Vec2 impulse, Vec2 point) {
        if (isSleeping() == false) {
            m_linearVelocity.x += m_invMass * impulse.x;
            m_linearVelocity.y += m_invMass * impulse.y;
            m_angularVelocity += m_invI * Vec2.cross(point.sub(m_position), impulse);
        }
    }
    
    public float GetMass() {
        return m_mass;
    }
    
    public float GetInertia() {
        return m_I;
    }
    
    public Vec2 GetWorldPoint(Vec2 localPoint) {
        return m_position.add(m_R.mul(localPoint));
    }
    
    public Vec2 GetWorldVector(Vec2 localVector) {
        return m_R.mul(localVector);
    }
    
    public Vec2 GetLocalPoint(Vec2 worldPoint) {
        return m_R.mulT(worldPoint.sub(m_position));
    }
    
    public Vec2 GetLocalVector(Vec2 worldVector) {
        return m_R.mulT(worldVector);
    }
    
    public boolean IsStatic() {
        return (m_flags & e_staticFlag) == e_staticFlag;
    }
    
    public boolean IsFrozen() {
        return (m_flags & e_frozenFlag) == e_frozenFlag;
    }
    
    public boolean IsSleeping() {
        return (m_flags & e_sleepFlag) == e_sleepFlag;
    }
    
    public void AllowSleeping(boolean flag) {
        if (flag) {
            m_flags |= e_allowSleepFlag;
        } else {
            m_flags &= ~e_allowSleepFlag;
            wakeUp();
        }
    }
    
    public Shape GetShapeList() {
        return m_shapeList;
    }
    
    public ContactNode GetContactList() {
        return m_contactList;
    }
    
    public JointNode GetJointList() {
        return m_jointList;
    }
    
    public Body GetNext() {
        return m_next;
    }
    
    public Object GetUserData() {
        return m_userData;
    }
    
    Body(BodyDef bd, World world) {
        m_flags = 0;
        m_position = bd.position.clone();
        m_rotation = bd.rotation;
        m_R = new Mat22(m_rotation);
        //m_linearVelocity = bd.linearVelocity.clone();
        //m_angularVelocity = bd.angularVelocity;
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
        } else {
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
        } else {
            m_invMass = 0.0f;
        }

        if (m_I > 0.0f && bd.preventRotation == false) {
            m_invI = 1.0f / m_I;
        } else {
            m_I = 0.0f;
            m_invI = 0.0f;
        }
        
        // Compute the center of mass velocity.
        m_linearVelocity = bd.linearVelocity.add(Vec2.cross(bd.angularVelocity, m_center));
        m_angularVelocity = bd.angularVelocity;

        m_jointList = null;
        m_contactList = null;
        m_prev = null;
        m_next = null;

        // Create the shapes.
        m_shapeList = null;
        for (int i = 0; i < m_shapeCount; ++i) {
            ShapeDef sd = bd.shapes[i];
            Shape shape = Shape.Create(sd, this, m_center);
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
        
        if ( ((m_flags & e_sleepFlag)>0) || m_invMass == 0.0f) {
            m_linearVelocity.set(0.0f, 0.0f);
            m_angularVelocity = 0.0f;
        }
        
        m_userData = bd.userData;
    }
    
    public void Destructor() {
        Shape s = m_shapeList;
        while (s != null) {
            Shape s0 = s;
            s = s.m_next;
            Shape.Destroy(s0);
        }
    }

    public void SetOriginPosition(Vec2 position, float rotation) {
        if (IsFrozen()){
            return;
        }
        
        m_rotation = rotation;
        m_R.set(m_rotation);
        m_position = position.add(m_R.mul(m_center));
        
        for (Shape s = m_shapeList; s != null; s = s.m_next) {
            s.Synchronize(m_position,m_R);
        }
        
        m_world.m_broadPhase.Flush();
    }

    public void SetCenterPosition(Vec2 position, float rotation) {
        if (IsFrozen()){
            return;
        }
        
        m_rotation = rotation;
        m_R.set(m_rotation);
        m_position = position.clone();
        
        for (Shape s = m_shapeList; s != null; s = s.m_next) {
            s.Synchronize(m_position,m_R);
        }
        
        m_world.m_broadPhase.Flush();
    }
    
    void SynchronizeShapes() {
        for (Shape s = m_shapeList; s != null; s = s.m_next) {
            s.Synchronize(m_position,m_R);
        }
    }
    
    public void Freeze() {
        m_flags |= e_frozenFlag;
        m_linearVelocity.setZero();
        m_angularVelocity = 0.0f;
    }

    boolean isSleeping() {
        return (m_flags & e_sleepFlag) == e_sleepFlag;
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

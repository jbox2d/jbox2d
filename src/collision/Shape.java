package collision;

import common.Mat22;
import common.Vec2;

import dynamics.Body;

public abstract class Shape {
    public int uid; // unique id for shape for sorting

    static private int uidcount = 0;

    public ShapeType m_type;

    public Body m_body;

    int m_proxyId;

    // Position in world
    public Vec2 m_position;

    public float m_rotation;

    public Mat22 m_R;

    // Local position in parent body
    public Vec2 m_localPosition;

    public float m_localRotation;

    public float m_friction;

    public float m_restitution;

    public Shape m_next;
    
    public Object m_userData;
    
    public Vec2 GetPosition(){
        return m_position;
    }
    
    public Mat22 GetRotationMatrix(){
        return m_R;
    }
    
    public ShapeType GetType() {
        return m_type;
    }
    
    public Object GetUserData() {
        return m_userData;
    }
    
    public Body GetBody() {
        return m_body;
    }
    
    public Shape GetNext() {
        return m_next;
    }

    //public abstract void UpdateProxy();
    public abstract void Synchronize(Vec2 position, Mat22 R);
    
    public abstract void ResetProxy(BroadPhase broadPhase);

    public abstract boolean TestPoint(Vec2 p);

    public static Shape Create(ShapeDef description, Body body,
            Vec2 center) {

        if (description.type == ShapeType.CIRCLE_SHAPE) {
            return new CircleShape(description, body, center);
        }
        else if (description.type == ShapeType.BOX_SHAPE
                || description.type == ShapeType.POLY_SHAPE) {
            return new PolyShape(description, body, center);
        }
        return null;
    }
    
    public static void Destroy(Shape shape) {
        shape.Destructor();
    }
    
    public void Destructor() {
        if (m_proxyId != PairManager.NULL_PROXY) {
            m_body.m_world.m_broadPhase.DestroyProxy(m_proxyId);
        }
    }

    public Shape(ShapeDef description, Body body, Vec2 center) {
        m_localPosition = new Vec2();//description.localPosition.sub(center);
        //m_localRotation = description.localRotation;
        m_friction = description.friction;
        m_restitution = description.restitution;
        m_body = body;

        m_position = new Vec2();//m_body.m_position.add(m_body.m_R.mul(m_localPosition));
        //m_rotation = m_body.m_rotation + m_localRotation;
        m_R = new Mat22();//(m_rotation);

        m_proxyId = PairManager.NULL_PROXY;
        uid = uidcount++;
    }

}

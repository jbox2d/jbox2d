package collision;

import common.Vec2;
import common.Mat22;
import dynamics.Body;

public class CircleShape extends Shape {

    public float m_radius;

    CircleShape(ShapeDef def, Body body, Vec2 localCenter) {
        super(def, body, localCenter);

        // b2Assert(description->type == e_circleShape);
        CircleDef circle = (CircleDef)def;
        
        m_localPosition = def.localPosition.sub(localCenter);
        m_type = ShapeType.CIRCLE_SHAPE;
        m_radius = circle.radius;
        m_R = m_body.m_R.clone();
        m_position = m_body.m_position.add(m_body.m_R.mul(m_localPosition));

        AABB aabb = new AABB(new Vec2(m_position.x - m_radius, m_position.y
                - m_radius), new Vec2(m_position.x + m_radius, m_position.y
                + m_radius));
        BroadPhase broadPhase = m_body.m_world.m_broadPhase;
        if (broadPhase.InRange(aabb)) {
            m_proxyId = broadPhase.CreateProxy(aabb, def.groupIndex, def.categoryBits, def.maskBits, this);
        } else {
            m_proxyId = PairManager.NULL_PROXY;
        }
        
        if (m_proxyId == PairManager.NULL_PROXY) {
            m_body.Freeze();
        }
    }

    public void Synchronize(Vec2 position, Mat22 R) {
        m_R.set(R);
        m_position = position.add(R.mul(m_localPosition));

        if (m_proxyId == PairManager.NULL_PROXY) {   
            return;
        }

        AABB aabb = new AABB();
        aabb.minVertex.set(m_position.x - m_radius, m_position.y - m_radius);
        aabb.maxVertex.set(m_position.x + m_radius, m_position.y + m_radius);

        BroadPhase broadPhase = m_body.m_world.m_broadPhase;
        if (broadPhase.InRange(aabb)) {
            broadPhase.MoveProxy(m_proxyId, aabb);
        } else {
            broadPhase.DestroyProxy(m_proxyId);
            m_proxyId = PairManager.NULL_PROXY;
            m_body.Freeze();
        }
    }
    
    @Override
    public boolean TestPoint(Vec2 p) {
        Vec2 d = p.sub(m_position);
        return Vec2.dot(d, d) <= m_radius * m_radius;
    }

    
    
    public void ResetProxy(BroadPhase broadPhase) {
        if (m_proxyId == PairManager.NULL_PROXY) {   
            return;
        }

        Proxy proxy = broadPhase.GetProxy(m_proxyId);
        int groupIndex = proxy.groupIndex;
        int categoryBits = proxy.categoryBits;
        int maskBits = proxy.maskBits;

        broadPhase.DestroyProxy(m_proxyId);
        proxy = null;

        AABB aabb = new AABB();
        aabb.minVertex.set(m_position.x - m_radius, m_position.y - m_radius);
        aabb.maxVertex.set(m_position.x + m_radius, m_position.y + m_radius);

        if (broadPhase.InRange(aabb)) {
            m_proxyId = broadPhase.CreateProxy(aabb, groupIndex, categoryBits, maskBits, this);
        } else {
            m_proxyId = PairManager.NULL_PROXY;
        }

        if (m_proxyId == PairManager.NULL_PROXY) {
            m_body.Freeze();
        }
    }
    
}

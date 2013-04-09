package collision;

import common.Vec2;
import dynamics.Body;

public class CircleShape extends Shape {

    float m_radius;

    CircleShape(ShapeDescription description, Body body, Vec2 center) {
        super(description, body, center);

        // b2Assert(description->type == e_circleShape);
        m_type = ShapeType.CIRCLE_SHAPE;
        m_radius = description.circle.m_radius;

        AABB aabb = new AABB(new Vec2(m_position.x - m_radius, m_position.y
                - m_radius), new Vec2(m_position.x + m_radius, m_position.y
                + m_radius));

        m_body.m_world.m_broadPhase.CreateProxy(aabb, this);
    }

    @Override
    public boolean TestPoint(Vec2 p) {
        Vec2 d = p.sub(m_position);
        return Vec2.dot(d, d) <= m_radius * m_radius;
    }

    @Override
    public void UpdateProxy() {
        AABB aabb = new AABB(new Vec2(m_position.x - m_radius, m_position.y
                - m_radius), new Vec2(m_position.x + m_radius, m_position.y
                + m_radius));

        m_body.m_world.m_broadPhase.MoveProxy(m_proxyId, aabb);
    }
}

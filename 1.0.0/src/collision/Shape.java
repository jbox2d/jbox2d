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

    public abstract void UpdateProxy();

    public abstract boolean TestPoint(Vec2 p);

    public static Shape Create(ShapeDescription description, Body body,
            Vec2 center, MassData massData) {

        if (description.type == ShapeType.CIRCLE_SHAPE) {
            return new CircleShape(description, body, center);
        }
        else if (description.type == ShapeType.BOX_SHAPE
                || description.type == ShapeType.POLY_SHAPE) {
            return new PolyShape(description, body, center, massData);
        }
        // TODO
        return null;
    }

    Shape(ShapeDescription description, Body body, Vec2 center) {
        m_localPosition = description.localPosition.sub(center);
        m_localRotation = description.localRotation;
        m_friction = description.friction;
        m_restitution = description.restitution;
        m_body = body;

        m_position = m_body.m_position.add(m_body.m_R.mul(m_localPosition));
        m_rotation = m_body.m_rotation + m_localRotation;
        m_R = new Mat22(m_rotation);

        m_proxyId = PairManager.NULL_PROXY;
        uid = uidcount++;
    }

    // b2Shape::~b2Shape()
    // {
    // m_body->m_world->m_broadPhase->DestroyProxy(m_proxyId);
    // }

    // b2CircleShape::b2CircleShape(const b2ShapeDescription* description,
    // b2Body* body, const b2Vec2& center)
    // : b2Shape(description, body, center)
    // {
    // b2Assert(description->type == e_circleShape);
    // m_type = e_circleShape;
    // m_radius = description->circle.m_radius;
    //
    // b2AABB aabb;
    // aabb.minVertex.Set(m_position.x - m_radius, m_position.y - m_radius);
    // aabb.maxVertex.Set(m_position.x + m_radius, m_position.y + m_radius);
    //
    // m_body->m_world->m_broadPhase->CreateProxy(aabb, this);
    // }
    //
    // void b2CircleShape::UpdateProxy()
    // {
    // b2AABB aabb;
    // aabb.minVertex.Set(m_position.x - m_radius, m_position.y - m_radius);
    // aabb.maxVertex.Set(m_position.x + m_radius, m_position.y + m_radius);
    //
    // m_body->m_world->m_broadPhase->MoveProxy(m_proxyId, aabb);
    // }
    //
    // bool b2CircleShape::TestPoint(const b2Vec2& p)
    // {
    // b2Vec2 d = p - m_position;
    // return b2Dot(d, d) <= m_radius * m_radius;
    // }
    //
    // b2PolyShape::b2PolyShape(const b2ShapeDescription* description, b2Body*
    // body,
    // const b2Vec2& center, const b2MassData* massData)
    // : b2Shape(description, body, center)
    // {
    // b2Assert(description->type == e_boxShape || description->type ==
    // e_polyShape);
    // m_type = e_polyShape;
    //
    // if (description->type == e_boxShape)
    // {
    // m_vertexCount = 4;
    // b2Vec2 h = description->box.m_extents;
    // m_vertices[0].Set(h.x, h.y);
    // m_vertices[1].Set(-h.x, h.y);
    // m_vertices[2].Set(-h.x, -h.y);
    // m_vertices[3].Set(h.x, -h.y);
    // m_normals[0].Set(0.0f, 1.0f);
    // m_normals[1].Set(-1.0f, 0.0f);
    // m_normals[2].Set(0.0f, -1.0f);
    // m_normals[3].Set(1.0f, 0.0f);
    // m_next[0] = 1;
    // m_next[1] = 2;
    // m_next[2] = 3;
    // m_next[3] = 0;
    //
    // m_extents = h;
    // }
    // else
    // {
    // b2AABB aabb;
    // aabb.minVertex.Set(FLT_MAX, FLT_MAX);
    // aabb.maxVertex.Set(-FLT_MAX, -FLT_MAX);
    // m_vertexCount = description->poly.m_vertexCount;
    // b2Assert(3 <= m_vertexCount && m_vertexCount <= b2_maxPolyVertices);
    // for (int32 i = 0; i < m_vertexCount; ++i)
    // {
    // m_vertices[i] = description->poly.m_vertices[i];
    //
    // aabb.minVertex = b2Min(aabb.minVertex, description->poly.m_vertices[i]);
    // aabb.maxVertex = b2Max(aabb.maxVertex, description->poly.m_vertices[i]);
    // }
    // b2Vec2 offset = 0.5f * (aabb.minVertex + aabb.maxVertex);
    //
    // b2Assert(m_localRotation == 0.0f); // TODO_ERIN handle local rotation
    //
    // m_localPosition += offset;
    // for (int32 i = 0; i < m_vertexCount; ++i)
    // {
    // // Shift the vertices so the shape position is the centroid.
    // m_vertices[i] = description->poly.m_vertices[i] - offset;
    // m_next[i] = i + 1 < m_vertexCount ? i + 1 : 0;
    // b2Vec2 vNext = description->poly.m_vertices[m_next[i]] - offset;
    // b2Vec2 edge = vNext - m_vertices[i];
    // m_normals[i] = b2Cross(edge, 1.0f);
    // m_normals[i].Normalize();
    // }
    //
    // for (int32 i = 0; i < m_vertexCount; ++i)
    // {
    // // Ensure the polygon in convex.
    // b2Assert(b2Cross(m_normals[i], m_normals[m_next[i]]) > 0.0f);
    // }
    //
    // m_extents = 0.5f * (aabb.maxVertex - aabb.minVertex);
    // }
    //
    // b2Mat22 absR = b2Abs(m_R);
    // b2Vec2 h = b2Mul(absR, m_extents);
    // b2AABB aabb;
    // aabb.minVertex = m_position - h;
    // aabb.maxVertex = m_position + h;
    // m_proxyId = m_body->m_world->m_broadPhase->CreateProxy(aabb, this);
    // }
    //
    // void b2PolyShape::UpdateProxy()
    // {
    // b2Mat22 absR = b2Abs(m_R);
    // b2Vec2 h = b2Mul(absR, m_extents);
    // b2AABB aabb;
    // aabb.minVertex = m_position - h;
    // aabb.maxVertex = m_position + h;
    // m_body->m_world->m_broadPhase->MoveProxy(m_proxyId, aabb);
    // }
    //
    // bool b2PolyShape::TestPoint(const b2Vec2& p)
    // {
    // b2Vec2 pLocal = b2MulT(m_R, p - m_position);
    //
    // for (int32 i = 0; i < m_vertexCount; ++i)
    // {
    // float32 dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
    // if (dot > 0.0f)
    // {
    // return false;
    // }
    // }
    //
    // return true;
    // }
}

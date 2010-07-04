package collision;

import common.Mat22;
import common.Settings;
import common.Vec2;
import dynamics.Body;

public class PolyShape extends Shape {

    Vec2 m_extents;

    public Vec2 m_vertices[];

    public int m_vertexCount;

    Vec2 m_normals[];

    int m_next[];

    PolyShape(ShapeDescription description, Body body, Vec2 center,
            MassData massData) {
        super(description, body, center);

        m_vertices = new Vec2[Settings.maxPolyVertices];
        m_normals = new Vec2[Settings.maxPolyVertices];
        m_next = new int[Settings.maxPolyVertices];

        m_type = ShapeType.POLY_SHAPE;// description.type;//
                                        // ShapeType.BOX_SHAPE; //TODO: hmm? why
                                        // BOX_SHAPE?

        if (description.type == ShapeType.BOX_SHAPE) {
            m_vertexCount = 4;
            Vec2 h = description.box.m_extents.clone();
            m_vertices[0] = new Vec2(h.x, h.y);
            m_vertices[1] = new Vec2(-h.x, h.y);
            m_vertices[2] = new Vec2(-h.x, -h.y);
            m_vertices[3] = new Vec2(h.x, -h.y);
            m_normals[0] = new Vec2(0.0f, 1.0f);
            m_normals[1] = new Vec2(-1.0f, 0.0f);
            m_normals[2] = new Vec2(0.0f, -1.0f);
            m_normals[3] = new Vec2(1.0f, 0.0f);
            m_next[0] = 1;
            m_next[1] = 2;
            m_next[2] = 3;
            m_next[3] = 0;

            m_extents = h;
            // System.out.println(description.box.m_extents.y);
        }
        else {
            AABB aabb = new AABB(new Vec2(Float.MAX_VALUE, Float.MAX_VALUE),
                    new Vec2(-Float.MAX_VALUE, -Float.MAX_VALUE));
            m_vertexCount = description.poly.m_vertices.size();
            // b2Assert(3 <= m_vertexCount && m_vertexCount <=
            // b2_maxPolyVertices);
            for (int i = 0; i < m_vertexCount; ++i) {
                m_vertices[i] = description.poly.m_vertices.get(i);

                aabb.minVertex = Vec2.min(aabb.minVertex, m_vertices[i]);
                aabb.maxVertex = Vec2.max(aabb.maxVertex, m_vertices[i]);
            }

            // Vec2 offset = aabb.minVertex.add(aabb.maxVertex).mul(0.5f);
            Vec2 offset = aabb.minVertex.clone().addLocal(aabb.maxVertex)
                    .mulLocal(0.5f);

            assert m_localRotation == 0.0f;
            // TODO_ERIN handle local rotation

            m_localPosition.addLocal(offset);
            for (int i = 0; i < m_vertexCount; ++i) {
                // Shift the vertices so the shape position is the centroid.
                m_vertices[i] = description.poly.m_vertices.get(i).sub(offset);
                m_next[i] = i + 1 < m_vertexCount ? i + 1 : 0;
                Vec2 vNext = description.poly.m_vertices.get(m_next[i]).sub(
                        offset);
                Vec2 edge = vNext.sub(m_vertices[i]);
                m_normals[i] = Vec2.cross(edge, 1.0f);
                m_normals[i].normalize();
            }

            for (int i = 0; i < m_vertexCount; ++i) {
                // Ensure the polygon in convex.
                assert Vec2.cross(m_normals[i], m_normals[m_next[i]]) > 0.0f;
            }
            // System.out.println("branch else");
            m_extents = aabb.maxVertex.sub(aabb.minVertex).mul(0.5f);
        }

        Mat22 absR = m_R.abs();
        Vec2 h = absR.mul(m_extents);
        // System.out.printf("h: %f %f ; m_extents: %f %f \n",
        // h.x,h.y,m_extents.x,m_extents.y);
        AABB aabb = new AABB(m_position.sub(h), m_position.add(h));
        // System.out.println(aabb.minVertex.x + " " +aabb.minVertex.y + ";
        // "+aabb.maxVertex.x+" "+aabb.maxVertex.y);
        m_proxyId = m_body.m_world.m_broadPhase.CreateProxy(aabb, this);
    }

    @Override
    public boolean TestPoint(Vec2 p) {
        Vec2 pLocal = m_R.mulT(p.sub(m_position));

        for (int i = 0; i < m_vertexCount; ++i) {
            float dot = Vec2.dot(m_normals[i], pLocal.sub(m_vertices[i]));
            if (dot > 0.0f) {
                return false;
            }
        }

        return true;
    }

    @Override
    public void UpdateProxy() {
        Mat22 absR = m_R.abs();
        Vec2 h = absR.mul(m_extents);
        AABB aabb = new AABB(m_position.sub(h), m_position.add(h));
        m_body.m_world.m_broadPhase.MoveProxy(m_proxyId, aabb);
    }
}

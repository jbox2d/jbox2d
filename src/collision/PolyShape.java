package collision;

import common.Mat22;
import common.Settings;
import common.Vec2;

import dynamics.Body;

public class PolyShape extends Shape {

    OBB m_localOBB;

    public Vec2 m_vertices[];

    public int m_vertexCount;

    Vec2 m_normals[];

    int m_next[];

    public PolyShape(ShapeDef def, Body body, Vec2 localCenter) {
        super(def, body, localCenter);

        m_vertices = new Vec2[Settings.maxPolyVertices];
        m_normals = new Vec2[Settings.maxPolyVertices];
        m_next = new int[Settings.maxPolyVertices];

        m_type = ShapeType.POLY_SHAPE;
        Mat22 localR = new Mat22(def.localRotation);
        Vec2 localPosition = def.localPosition.sub(localCenter);
        if (def.type == ShapeType.BOX_SHAPE) {
            BoxDef box = (BoxDef) def;
            m_vertexCount = 4;
            Vec2 h = box.extents.clone();
            m_vertices[0] = localR.mul(new Vec2(h.x, h.y)).addLocal(localPosition);
            m_vertices[1] = localR.mul(new Vec2(-h.x, h.y)).addLocal(localPosition);
            m_vertices[2] = localR.mul(new Vec2(-h.x, -h.y)).addLocal(localPosition);
            m_vertices[3] = localR.mul(new Vec2(h.x, -h.y)).addLocal(localPosition);
        }
        else {
            PolyDef poly = (PolyDef) def;
            m_vertexCount = poly.vertices.size();
            // b2Assert(3 <= m_vertexCount && m_vertexCount <=
            // b2_maxPolyVertices);
            for (int i = 0; i < m_vertexCount; ++i) {
                m_vertices[i] = localR.mul(poly.vertices
                        .get(i)).addLocal(localPosition);
            }

        }

        // Compute bounding box. TODO_ERIN optimize OBB
        Vec2 minVertex = new Vec2(Float.MAX_VALUE, Float.MAX_VALUE);
        Vec2 maxVertex = new Vec2(-Float.MAX_VALUE, -Float.MAX_VALUE);
        for (int i = 0; i < m_vertexCount; ++i) {
            minVertex = Vec2.min(minVertex, m_vertices[i]);
            maxVertex = Vec2.max(maxVertex, m_vertices[i]);
        }

        m_localOBB = new OBB();
        m_localOBB.R.setIdentity();
        m_localOBB.center = new Vec2(.5f * (minVertex.x + maxVertex.x),
                .5f * (minVertex.y + maxVertex.y));
        m_localOBB.extents = new Vec2(.5f * (maxVertex.x - minVertex.x),
                .5f * (maxVertex.y - minVertex.y));

        // Compute the edge normals and next index map
        for (int i = 0; i < m_vertexCount; ++i) {
            m_next[i] = i + 1 < m_vertexCount ? i + 1 : 0;
            Vec2 edge = m_vertices[m_next[i]].sub(m_vertices[i]);
            m_normals[i] = Vec2.cross(edge, 1.0f);
            m_normals[i].normalize();
        }

        // Ensure the polygon is convex. TODO_ERIN compute convex hull
        for (int i = 0; i < m_vertexCount; ++i) {
            assert (Vec2.cross(m_normals[i], m_normals[m_next[i]]) > 0.0f);
        }

        // The body transform is copied for convenience
        m_R = m_body.m_R;
        m_position = m_body.m_position;

        Mat22 R = m_R.mul(m_localOBB.R);
        Mat22 absR = R.abs();
        Vec2 h = absR.mul(m_localOBB.extents);
        Vec2 position = m_R.mul(m_localOBB.center).addLocal(m_position);
        AABB aabb = new AABB();
        aabb.minVertex.x = position.x - h.x;
        aabb.minVertex.y = position.y - h.y;
        aabb.maxVertex.x = position.x + h.x;
        aabb.maxVertex.y = position.y + h.y;

        BroadPhase broadPhase = m_body.m_world.m_broadPhase;
        if (broadPhase.inRange(aabb)) {
            m_proxyId = broadPhase.CreateProxy(aabb, def.groupIndex,
                    def.categoryBits, def.maskBits, this);
        }
        else {
            m_proxyId = PairManager.NULL_PROXY;
        }

        if (m_proxyId == PairManager.NULL_PROXY) {
            m_body.freeze();
        }

    }

    public void synchronize(Vec2 position, Mat22 R) {
        m_R = R;
        m_position = position;

        if (m_proxyId == PairManager.NULL_PROXY) {
            return;
        }

        Mat22 obbR = m_R.mul(m_localOBB.R);
        Mat22 absR = obbR.abs();
        Vec2 h = absR.mul(m_localOBB.extents);
        Vec2 center = m_R.mul(m_localOBB.center).addLocal(m_position);
        AABB aabb = new AABB();
        aabb.minVertex.x = center.x - h.x;
        aabb.minVertex.y = center.y - h.y;
        aabb.maxVertex.x = center.x + h.x;
        aabb.maxVertex.y = center.y + h.y;

        BroadPhase broadPhase = m_body.m_world.m_broadPhase;
        if (broadPhase.inRange(aabb)) {
            broadPhase.moveProxy(m_proxyId, aabb);
        }
        else {
            broadPhase.destroyProxy(m_proxyId);
            m_proxyId = PairManager.NULL_PROXY;
            m_body.freeze();
        }

    }

    @Override
    public boolean testPoint(Vec2 p) {
        Vec2 pLocal = m_R.mulT(p.sub(m_position));

        for (int i = 0; i < m_vertexCount; ++i) {
            float dot = Vec2.dot(m_normals[i], pLocal.sub(m_vertices[i]));
            if (dot > 0.0f) {
                return false;
            }
        }

        return true;
    }

    public void resetProxy(BroadPhase broadPhase) {
        if (m_proxyId == PairManager.NULL_PROXY) {
            return;
        }
        Proxy proxy = broadPhase.getProxy(m_proxyId);
        int groupIndex = proxy.groupIndex;
        int categoryBits = proxy.categoryBits;
        int maskBits = proxy.maskBits;

        broadPhase.destroyProxy(m_proxyId);
        proxy = null;

        Mat22 R = m_R.mul(m_localOBB.R);
        Mat22 absR = R.abs();
        Vec2 h = absR.mul(m_localOBB.extents);
        Vec2 position = m_R.mul(m_localOBB.center).addLocal(m_position);
        AABB aabb = new AABB();
        aabb.minVertex.x = position.x - h.x;
        aabb.minVertex.y = position.y - h.y;
        aabb.maxVertex.x = position.x + h.x;
        aabb.maxVertex.y = position.y + h.y;

        if (broadPhase.inRange(aabb)) {
            m_proxyId = broadPhase.CreateProxy(aabb, groupIndex, categoryBits,
                    maskBits, this);
        }
        else {
            m_proxyId = PairManager.NULL_PROXY;
        }

        if (m_proxyId == PairManager.NULL_PROXY) {
            m_body.freeze();
        }
    }

}

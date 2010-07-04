/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.gphysics.com
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
package collision;

import common.*;

import dynamics.Body;

//Updated to rev 56 of b2Shape.cpp/.h

public class PolyShape extends Shape {

    OBB m_localOBB;

    public Vec2 m_vertices[];

    public int m_vertexCount;

    Vec2 m_normals[];

    // Local position of the shape centroid in parent body frame.
    public Vec2 m_localCentroid;
    
    public Vec2 m_coreVertices[];

    public PolyShape(ShapeDef def, Body body, Vec2 newOrigin) {
        super(def, body);

        m_vertices = new Vec2[Settings.maxPolyVertices];
        m_normals = new Vec2[Settings.maxPolyVertices];
        m_coreVertices = new Vec2[Settings.maxPolyVertices];
        
        assert(def.type == ShapeType.BOX_SHAPE || def.type == ShapeType.POLY_SHAPE);
        m_type = ShapeType.POLY_SHAPE;
        Mat22 localR = new Mat22(def.localRotation);

        if (def.type == ShapeType.BOX_SHAPE) {
            m_localCentroid = def.localPosition.sub(newOrigin);

            BoxDef box = (BoxDef) def;
            m_vertexCount = 4;
            Vec2 h = box.extents.clone();
            Vec2 hc = h.clone();
            hc.x = Math.max(0.0f, h.x - 2.0f * Settings.linearSlop);
            hc.y = Math.max(0.0f, h.y - 2.0f * Settings.linearSlop);
            m_vertices[0] = localR.mul(new Vec2(h.x, h.y));
            m_vertices[1] = localR.mul(new Vec2(-h.x, h.y));
            m_vertices[2] = localR.mul(new Vec2(-h.x, -h.y));
            m_vertices[3] = localR.mul(new Vec2(h.x, -h.y));
            
            m_coreVertices[0] = localR.mul(new Vec2(hc.x, hc.y));
            m_coreVertices[1] = localR.mul(new Vec2(-hc.x, hc.y));
            m_coreVertices[2] = localR.mul(new Vec2(-hc.x, -hc.y));
            m_coreVertices[3] = localR.mul(new Vec2(hc.x, -hc.y));
        } else {
            PolyDef poly = (PolyDef) def;
            m_vertexCount = poly.vertices.size();
            assert(3 <= m_vertexCount && m_vertexCount <= Settings.maxPolyVertices);
            Vec2 centroid = ShapeDef.polyCentroid(poly.vertices, poly.vertices.size());
            m_localCentroid = localR.mul(centroid).addLocal(def.localPosition).subLocal(newOrigin);
            for (int i = 0; i < m_vertexCount; ++i) {
                m_vertices[i] = localR.mul(poly.vertices.get(i).sub(centroid));

                Vec2 u = m_vertices[i].clone();
                float length = u.length();
                if (length > Settings.EPSILON) {
                    u.mulLocal(1.0f / length);
                }
                
                //m_coreVertices[i] = m_vertices[i] - 2.0f * b2_linearSlop * u;
                m_coreVertices[i] = u.mulLocal(-Settings.linearSlop*2.0f).addLocal(m_vertices[i]);

            }

        }

        // Compute bounding box. TODO_ERIN optimize OBB
        Vec2 minVertex = new Vec2(Float.MAX_VALUE, Float.MAX_VALUE);
        Vec2 maxVertex = new Vec2(-Float.MAX_VALUE, -Float.MAX_VALUE);
        m_maxRadius = 0f;
        for (int i = 0; i < m_vertexCount; ++i) {
            Vec2 v = m_vertices[i];
            minVertex = Vec2.min(minVertex, v);
            maxVertex = Vec2.max(maxVertex, v);
            m_maxRadius = Math.max(m_maxRadius, v.length());
        }

        m_localOBB = new OBB();
        m_localOBB.R.setIdentity();
        m_localOBB.center = new Vec2(.5f * (minVertex.x + maxVertex.x),
                .5f * (minVertex.y + maxVertex.y));
        m_localOBB.extents = new Vec2(.5f * (maxVertex.x - minVertex.x),
                .5f * (maxVertex.y - minVertex.y));

        // Compute the edge normals and next index map
        for (int i = 0; i < m_vertexCount; ++i) {
            int i1 = i;
            int i2 = i+1 < m_vertexCount ? i+1 : 0;
            Vec2 edge = m_vertices[i2].sub(m_vertices[i1]);
            m_normals[i] = Vec2.cross(edge, 1.0f);
            m_normals[i].normalize();
        }

        // Ensure the polygon is convex. TODO_ERIN compute convex hull
        for (int i = 0; i < m_vertexCount; ++i) {
            int i1 = i;
            int i2 = i+1 < m_vertexCount ? i+1 : 0;
            //ewjordan: I think this should check for the cross
            //to be >= 0 - there are some situations where the
            //EPSILON check will fail under convex decomposition
            assert (Vec2.cross(m_normals[i1], m_normals[i2]) > Settings.EPSILON) : "Possibly non-convex polygon";
        }

        // The body transform is copied for convenience
        m_R = m_body.m_R;
        m_position = m_body.m_position.add(m_body.m_R.mul(m_localCentroid));

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
            m_proxyId = broadPhase.CreateProxy(aabb, this);
        }
        else {
            m_proxyId = PairManager.NULL_PROXY;
        }

        if (m_proxyId == PairManager.NULL_PROXY) {
            m_body.freeze();
        }

    }

    public void synchronize(Vec2 position1, Mat22 R1, Vec2 position2, Mat22 R2) {
        m_R = R2;
        m_position = R2.mul(m_localCentroid).addLocal(position2);

        if (m_proxyId == PairManager.NULL_PROXY) {
            return;
        }

        AABB aabb1 = new AABB();
        AABB aabb2 = new AABB();

        {
            Mat22 obbR = R1.mul(m_localOBB.R);
            Mat22 absR = obbR.abs();
            Vec2 h = absR.mul(m_localOBB.extents);
            Vec2 center = R1.mul(m_localCentroid.add(m_localOBB.center)).addLocal(position1);
            aabb1.minVertex = center.sub(h);
            aabb1.maxVertex = center.add(h);
        }

        {
            Mat22 obbR = R2.mul(m_localOBB.R);
            Mat22 absR = obbR.abs();
            Vec2 h = absR.mul(m_localOBB.extents);
            Vec2 center = R2.mul(m_localCentroid.add(m_localOBB.center)).addLocal(position2);
            aabb2.minVertex = center.sub(h);
            aabb2.maxVertex = center.add(h);
        }

        AABB aabb = new AABB();
        aabb.minVertex = Vec2.min(aabb1.minVertex, aabb2.minVertex);
        aabb.maxVertex = Vec2.max(aabb1.maxVertex, aabb2.maxVertex);

        BroadPhase broadPhase = m_body.m_world.m_broadPhase;
        if (broadPhase.inRange(aabb))
        {
            broadPhase.moveProxy(m_proxyId, aabb);
        } else {
            m_body.freeze();
        }

    }
    
    public void quickSync(Vec2 position, Mat22 R) {
        m_R = R.clone();
        m_position = R.mul(m_localCentroid).addLocal(position); 
    }

    public Vec2 support(Vec2 d) {
        Vec2 dLocal = m_R.mulT(d);

        int bestIndex = 0;
        float bestValue = Vec2.dot(m_coreVertices[0], dLocal);
        for (int i = 1; i < m_vertexCount; ++i) {
            float value = Vec2.dot(m_coreVertices[i], dLocal);
            if (value > bestValue) {
                bestIndex = i;
                bestValue = value;
            }
        }

        return m_R.mul(m_coreVertices[bestIndex]).addLocal(m_position); 
        
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
        //Proxy proxy = broadPhase.getProxy(m_proxyId);

        broadPhase.destroyProxy(m_proxyId);
        //proxy = null;

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
            m_proxyId = broadPhase.CreateProxy(aabb, this);
        } else {
            m_proxyId = PairManager.NULL_PROXY;
        }

        if (m_proxyId == PairManager.NULL_PROXY) {
            m_body.freeze();
        }
    }

}

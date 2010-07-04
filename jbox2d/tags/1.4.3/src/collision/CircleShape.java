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

public class CircleShape extends Shape {

    public float m_radius;

    public CircleShape(ShapeDef def, Body body, Vec2 localCenter) {
        super(def, body);

        assert def.type == ShapeType.CIRCLE_SHAPE;

        CircleDef circle = (CircleDef) def;

        m_localPosition = def.localPosition.sub(localCenter);
        m_type = ShapeType.CIRCLE_SHAPE;
        m_radius = circle.radius;
        
        m_R = m_body.m_R.clone();
        Vec2 r = m_body.m_R.mul(m_localPosition);
        m_position = m_body.m_position.add(r);
        m_maxRadius = r.length() + m_radius;

        AABB aabb = new AABB(new Vec2(m_position.x - m_radius, m_position.y
                - m_radius), new Vec2(m_position.x + m_radius, m_position.y
                + m_radius));

        BroadPhase broadPhase = m_body.m_world.m_broadPhase;

        if (broadPhase.inRange(aabb)) {
            m_proxyId = broadPhase.CreateProxy(aabb, this);
        } else {
            m_proxyId = PairManager.NULL_PROXY;
        }

        if (m_proxyId == PairManager.NULL_PROXY) {
            m_body.freeze();
        }
    }

    public void synchronize(Vec2 position1, Mat22 R1, Vec2 position2, Mat22 R2) {
        m_R.set(R2);
        m_position = m_R.mul(m_localPosition).addLocal(position2);

        if (m_proxyId == PairManager.NULL_PROXY) {
            return;
        }

        // Compute an AABB that covers the swept shape (may miss some rotation effect).
        Vec2 p1 = R1.mul(m_localPosition).addLocal(position1);
        Vec2 lower = Vec2.min(p1, m_position);
        Vec2 upper = Vec2.max(p1, m_position);

        AABB aabb = new AABB();
        aabb.minVertex.set(lower.x - m_radius, lower.y - m_radius);
        aabb.maxVertex.set(upper.x + m_radius, upper.y + m_radius);

        BroadPhase broadPhase = m_body.m_world.m_broadPhase;
        if (broadPhase.inRange(aabb)) {
            broadPhase.moveProxy(m_proxyId, aabb);
        } else {
            m_body.freeze();
        }
    }
    
    public void quickSync(Vec2 position, Mat22 R) {
        m_R = R.clone();
        m_position = R.mul(m_localPosition).addLocal(position); 
    }

    public Vec2 support(Vec2 d) {
        Vec2 u = d.clone();
        u.normalize();
        float r = Math.max(0.0f, m_radius - 2.0f * Settings.linearSlop);
        return u.mulLocal(r).addLocal(m_position); 
    }

    @Override
    public boolean testPoint(Vec2 p) {
        Vec2 d = p.sub(m_position);
        return Vec2.dot(d, d) <= m_radius * m_radius;
    }

    public void resetProxy(BroadPhase broadPhase) {
        if (m_proxyId == PairManager.NULL_PROXY) {
            return;
        }

        //Proxy proxy = broadPhase.getProxy(m_proxyId); //don't bother

        broadPhase.destroyProxy(m_proxyId);
        //proxy = null; //totally ineffective, but harmless...

        AABB aabb = new AABB();
        aabb.minVertex.set(m_position.x - m_radius, m_position.y - m_radius);
        aabb.maxVertex.set(m_position.x + m_radius, m_position.y + m_radius);

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

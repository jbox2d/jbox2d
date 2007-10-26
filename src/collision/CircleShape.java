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

import common.Vec2;
import common.Mat22;
import dynamics.Body;

public class CircleShape extends Shape {

    public float m_radius;

    public CircleShape(ShapeDef def, Body body, Vec2 localCenter) {
        super(def, body, localCenter);

        assert def.type == ShapeType.CIRCLE_SHAPE;

        CircleDef circle = (CircleDef) def;

        m_localPosition = def.localPosition.sub(localCenter);
        m_type = ShapeType.CIRCLE_SHAPE;
        m_radius = circle.radius;
        m_R = m_body.m_R.clone();
        m_position = m_body.m_R.mul(m_localPosition).addLocal(m_body.m_position);

        AABB aabb = new AABB(new Vec2(m_position.x - m_radius, m_position.y
                - m_radius), new Vec2(m_position.x + m_radius, m_position.y
                + m_radius));

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
        m_R.set(R);
        m_position = R.mul(m_localPosition).addLocal(position);

        if (m_proxyId == PairManager.NULL_PROXY) {
            return;
        }

        AABB aabb = new AABB();
        aabb.minVertex.set(m_position.x - m_radius, m_position.y - m_radius);
        aabb.maxVertex.set(m_position.x + m_radius, m_position.y + m_radius);

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
        Vec2 d = p.sub(m_position);
        return Vec2.dot(d, d) <= m_radius * m_radius;
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

        AABB aabb = new AABB();
        aabb.minVertex.set(m_position.x - m_radius, m_position.y - m_radius);
        aabb.maxVertex.set(m_position.x + m_radius, m_position.y + m_radius);

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

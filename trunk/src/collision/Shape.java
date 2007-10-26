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

    public Shape(ShapeDef description, Body body, Vec2 center) {
        m_localPosition = new Vec2();// description.localPosition.sub(center);
        // m_localRotation = description.localRotation;
        m_friction = description.friction;
        m_restitution = description.restitution;
        m_body = body;

        m_position = new Vec2();// m_body.m_position.add(m_body.m_R.mul(m_localPosition));
        // m_rotation = m_body.m_rotation + m_localRotation;
        m_R = new Mat22();// (m_rotation);

        m_proxyId = PairManager.NULL_PROXY;
        uid = uidcount++;
    }

    public Vec2 getPosition() {
        return m_position;
    }

    public Mat22 getRotationMatrix() {
        return m_R;
    }

    public ShapeType getType() {
        return m_type;
    }

    public Object getUserData() {
        return m_userData;
    }

    public Body getBody() {
        return m_body;
    }

    public Shape getNext() {
        return m_next;
    }

    // public abstract void UpdateProxy();
    public abstract void synchronize(Vec2 position, Mat22 R);

    public abstract void resetProxy(BroadPhase broadPhase);

    public abstract boolean testPoint(Vec2 p);

    public static Shape create(ShapeDef description, Body body, Vec2 center) {

        if (description.type == ShapeType.CIRCLE_SHAPE) {
            return new CircleShape(description, body, center);
        }
        else if (description.type == ShapeType.BOX_SHAPE
                || description.type == ShapeType.POLY_SHAPE) {
            return new PolyShape(description, body, center);
        }
        return null;
    }

    public void destructor() {
        if (m_proxyId != PairManager.NULL_PROXY) {
            m_body.m_world.m_broadPhase.destroyProxy(m_proxyId);
        }
    }
}

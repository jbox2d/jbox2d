/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.box2d.org
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

package org.jbox2d.collision;

import java.util.List;

import org.jbox2d.common.*;


public class ShapeDef {
	// Holds the shape type for down-casting.
    public ShapeType type;
    
    // Use this to store application specify shape data.
    public Object userData;
    
    // The shape's friction coefficient, usually in the range [0,1].
    public float friction;
    
    // The shape's restitution (elasticity) usually in the range [0,1].
    public float restitution;

    // The shape's density, usually in kg/m^2.
    public float density;

    // The collision category bits. Normally you would just set one bit.
    public int categoryBits;

    // The collision mask bits. This states the categories that this
	// shape would accept for collision.
    public int maskBits;
    
    // Collision groups allow a certain group of objects to never collide (negative)
	// or always collide (positive). Zero means no collision group. Non-zero group
	// filtering always wins against the mask bits.
    public int groupIndex;
    
    // A sensor shape collects contact information but never generates a collision
	// response.
	public boolean isSensor;
    
    public ShapeDef() {

        type = ShapeType.UNKNOWN_SHAPE;
        userData = null;
        friction = 0.2f;
        restitution = 0.0f;
        density = 0.0f;
        categoryBits = 0x0001;
        maskBits = 0xFFFF;
        groupIndex = 0;
        isSensor = false;
        
    }

//    public void computeMass(MassData massData) {
//        if (density == 0.0f) {
//            massData.mass = 0.0f;
//            massData.center.set(0.0f, 0.0f);
//            massData.I = 0.0f;
//        }
//
//        switch (type) {
//        case CIRCLE_SHAPE:
//            CircleDef circle = (CircleDef) this;
//            massData.mass = (float) (density * Math.PI * circle.radius * circle.radius);
//            massData.center.set(0.0f, 0.0f);
//            massData.I = 0.5f * (massData.mass) * circle.radius * circle.radius;
//            break;
//
//        case BOX_SHAPE:
//            BoxDef box = (BoxDef) this;
//            massData.mass = 4.0f * density * box.extents.x * box.extents.y;
//            massData.center.set(0.0f, 0.0f);
//            massData.I = massData.mass / 3.0f
//                    * Vec2.dot(box.extents, box.extents);
//            break;
//
//        case POLY_SHAPE:
//            PolygonDef poly = (PolygonDef) this;
//            polyMass(massData, poly.vertices, density);
//            break;
//
//        default:
//            massData.mass = 0.0f;
//            massData.center.set(0.0f, 0.0f);
//            massData.I = 0.0f;
//            break;
//        }
//    }

//    private void polyMass(MassData massData, List<Vec2> vs, float rho) {
//        int count = vs.size();
//
//        assert count >= 3;
//
//        Vec2 center = new Vec2(0.0f, 0.0f);
//        float area = 0.0f;
//        float I = 0.0f;
//
//        Vec2 pRef = new Vec2(0.0f, 0.0f);
//
//        // #if 0 XXX ?
//        // for (int i = 0; i < count; ++i) {
//        // pRef.addLocal(vs.get(i));
//        // }
//        // pRef.mulLocal(1.0f / count);
//        // #endif XXX ?
//
//        final float inv3 = 1.0f / 3.0f;
//
//        for (int i = 0; i < vs.size(); ++i) {
//            // Triangle vertices.
//            Vec2 p1 = pRef.clone();
//            Vec2 p2 = vs.get(i);
//            Vec2 p3 = i + 1 < count ? vs.get(i + 1) : vs.get(0);
//
//            Vec2 e1 = p2.sub(p1);
//            Vec2 e2 = p3.sub(p1);
//
//            float D = Vec2.cross(e1, e2);
//
//            float triangleArea = 0.5f * D;
//            area += triangleArea;
//
//            // Area weighted centroid
//            // center.addLocal(p1.add(p2).add(p3).mul(triangleArea * inv3));
//            center.addLocal(p1.clone().addLocal(p2).addLocal(p3).mul(
//                    triangleArea * inv3));
//
//            float px = p1.x, py = p1.y;
//            float ex1 = e1.x, ey1 = e1.y;
//            float ex2 = e2.x, ey2 = e2.y;
//
//            float intx2 = inv3
//                    * (0.25f * (ex1 * ex1 + ex2 * ex1 + ex2 * ex2) + (px * ex1 + px
//                            * ex2)) + 0.5f * px * px;
//            float inty2 = inv3
//                    * (0.25f * (ey1 * ey1 + ey2 * ey1 + ey2 * ey2) + (py * ey1 + py
//                            * ey2)) + 0.5f * py * py;
//
//            I += D * (intx2 + inty2);
//        }
//
//        // Total mass
//        massData.mass = rho * area;
//
//        // Center of mass
//        assert(area > Settings.EPSILON);
//        center.mulLocal(1.0f / area);
//        massData.center = center;
//
//        // Inertia tensor relative to the center.
//        I = rho * (I - area * Vec2.dot(center, center));
//        massData.I = I;
//    }
}
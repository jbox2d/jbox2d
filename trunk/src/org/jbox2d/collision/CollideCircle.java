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

import org.jbox2d.common.*;

//Updated to rev 108 of b2CollideCircle.cpp

public class CollideCircle {

    public static void collideCircle(Manifold manifold, 
    		CircleShape circle1, XForm xf1,
            CircleShape circle2, XForm xf2) {
        manifold.pointCount = 0;

    	Vec2 p1 = XForm.mul(xf1, circle1.m_localPosition);
    	Vec2 p2 = XForm.mul(xf2, circle2.m_localPosition);

    	Vec2 d = p2.sub(p1);
        
    	float distSqr = Vec2.dot(d, d);
        float radiusSum = circle1.m_radius + circle2.m_radius;
        if (distSqr > radiusSum * radiusSum) {
            return;
        }

        float separation;
        if (distSqr < Settings.EPSILON) {
            separation = -radiusSum;
            manifold.normal.set(0.0f, 1.0f);
        }
        else {
            float dist = (float) Math.sqrt(distSqr);
            separation = dist - radiusSum;
            float a = 1.0f / dist;
            manifold.normal.x = a * d.x;
            manifold.normal.y = a * d.y;
        }

        manifold.pointCount = 1;
        //manifold.points[0].id.key = 0;
        manifold.points[0].id.zero(); //use this instead of zeroing through key
        manifold.points[0].separation = separation;

    	p1.addLocal(manifold.normal.mul(circle1.m_radius));
    	p2.subLocal(manifold.normal.mul(circle2.m_radius));

    	Vec2 p = new Vec2(0.5f * (p1.x + p2.x), 0.5f * (p1.y + p2.y));

    	manifold.points[0].localPoint1 = XForm.mulT(xf1, p);
    	manifold.points[0].localPoint2 = XForm.mulT(xf2, p);

    }

    public static void collidePolygonAndCircle(Manifold manifold, 
    		PolygonShape polygon, XForm xf1,
            CircleShape circle, XForm xf2) {
    	
        manifold.pointCount = 0;

        // Compute circle position in the frame of the polygon.
        Vec2 c = XForm.mul(xf2, circle.m_localPosition);
    	Vec2 cLocal = XForm.mulT(xf1, c);

        // Find edge with maximum separation.
        int normalIndex = 0;
        float separation = -Float.MAX_VALUE;
        final float radius = circle.m_radius;
        for (int i = 0; i < polygon.m_vertexCount; ++i) {

            float s = Vec2.dot(polygon.m_normals[i], cLocal.sub(polygon.m_vertices[i]));
            if (s > circle.m_radius) {
                // Early out.
                return;
            }

            if (s > separation) {
                normalIndex = i;
                separation = s;
            }
        }
        // If the center is inside the polygon ...
        if (separation < Settings.EPSILON) {
            manifold.pointCount = 1;
            manifold.normal = Mat22.mul(xf1.R, polygon.m_normals[normalIndex]);
            manifold.points[0].id.features.incidentEdge = normalIndex;
            manifold.points[0].id.features.incidentVertex = Collision.NULL_FEATURE;
            manifold.points[0].id.features.referenceFace = Collision.NULL_FEATURE;
            manifold.points[0].id.features.flip = 0;
    		Vec2 position = c.sub(manifold.normal.mul(radius));
    		manifold.points[0].localPoint1 = XForm.mulT(xf1, position);
    		manifold.points[0].localPoint2 = XForm.mulT(xf2, position);
            manifold.points[0].separation = separation - radius;
            return;
        }

        // Project the circle center onto the edge segment.
        int vertIndex1 = normalIndex;
        int vertIndex2 = vertIndex1 + 1 < polygon.m_vertexCount ? vertIndex1 + 1 : 0;
        Vec2 e = polygon.m_vertices[vertIndex2].sub(polygon.m_vertices[vertIndex1]);
        float length = e.normalize();

        // If the edge length is zero ...
        if (length < Settings.EPSILON) {
            Vec2 d = cLocal.sub(polygon.m_vertices[vertIndex1]);
            float dist = d.normalize();
            if (dist > radius) {
                return;
            }

            manifold.pointCount = 1;
            manifold.normal = Mat22.mul(xf1.R, d);
            manifold.points[0].id.features.incidentEdge = Collision.NULL_FEATURE;
            manifold.points[0].id.features.incidentVertex = vertIndex1;
            manifold.points[0].id.features.referenceFace = Collision.NULL_FEATURE;
            manifold.points[0].id.features.flip = 0;
            Vec2 position = c.sub(manifold.normal.mul(radius));
    		manifold.points[0].localPoint1 = XForm.mulT(xf1, position);
    		manifold.points[0].localPoint2 = XForm.mulT(xf2, position);
            manifold.points[0].separation = dist - radius;
            return;
        }

        // Project the center onto the edge.
        float u = Vec2.dot(cLocal.sub(polygon.m_vertices[vertIndex1]), e);
        manifold.points[0].id.features.incidentEdge = Collision.NULL_FEATURE;
        manifold.points[0].id.features.incidentVertex = Collision.NULL_FEATURE;
        manifold.points[0].id.features.referenceFace = Collision.NULL_FEATURE;
        manifold.points[0].id.features.flip = 0;
        Vec2 p = new Vec2();
        if (u <= 0.0f) {
            p.set(polygon.m_vertices[vertIndex1]);
            manifold.points[0].id.features.incidentVertex = vertIndex1;
        }
        else if (u >= length) {
            p.set(polygon.m_vertices[vertIndex2]);
            manifold.points[0].id.features.incidentVertex = vertIndex2;
        }
        else {
            p.set(polygon.m_vertices[vertIndex1]);
            p.x += u * e.x;
            p.y += u * e.y;
            manifold.points[0].id.features.incidentEdge = vertIndex1;
        }

        Vec2 d = cLocal.sub(p);
        float dist = d.normalize();
        if (dist > radius) {
            return;
        }

        manifold.pointCount = 1;
        
    	manifold.normal = Mat22.mul(xf1.R, d);
    	Vec2 position = c.sub(manifold.normal.mul(radius));
    	manifold.points[0].localPoint1 = XForm.mulT(xf1, position);
    	manifold.points[0].localPoint2 = XForm.mulT(xf2, position);
        
        manifold.points[0].separation = dist - radius;
    }
}
/*
 * Copyright (c) 2007 Erin Catto http://www.gphysics.com
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

//Updated to rev 52 of b2CollideCircle.cpp

package collision;

import common.Settings;
import common.Vec2;

public class CollideCircle {

    public static void collideCircle(Manifold manifold, CircleShape circle1,
            CircleShape circle2, boolean conservative) {
        manifold.pointCount = 0;

        Vec2 d = circle2.m_position.sub(circle1.m_position);
        float distSqr = Vec2.dot(d, d);
        float radiusSum = circle1.m_radius + circle2.m_radius;
        if (distSqr > radiusSum * radiusSum && conservative == false) {
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
        manifold.points[0].separation = separation;
        manifold.points[0].position = circle2.m_position.sub(manifold.normal
                .mul(circle2.m_radius));
    }

    public static void collidePolyAndCircle(Manifold manifold, PolyShape poly,
            CircleShape circle, boolean conservative) {
        manifold.pointCount = 0;

        // Compute circle position in the frame of the polygon.
        Vec2 xLocal = poly.m_R.mulT(circle.m_position.sub(poly.m_position));

        // Find edge with maximum separation.
        int normalIndex = 0;
        float separation = -Float.MAX_VALUE;
        final float radius = circle.m_radius;
        for (int i = 0; i < poly.m_vertexCount; ++i) {
            float s = Vec2.dot(poly.m_normals[i], xLocal
                    .sub(poly.m_vertices[i]));
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
            // Java FIXME?: this reeks of needing initialization
            manifold.pointCount = 1;
            manifold.normal = poly.m_R.mul(poly.m_normals[normalIndex]);
            manifold.points[0].id.features.incidentEdge = normalIndex;
            manifold.points[0].id.features.incidentVertex = Collision.NULL_FEATURE;
            manifold.points[0].id.features.referenceFace = Collision.NULL_FEATURE;
            manifold.points[0].id.features.flip = 0;
            manifold.points[0].position = circle.m_position.sub(manifold.normal
                    .mul(radius));
            manifold.points[0].separation = separation - radius;
            return;
        }

        // Project the circle center onto the edge segment.
        int vertIndex1 = normalIndex;
        int vertIndex2 = vertIndex1 + 1 < poly.m_vertexCount ? vertIndex1 + 1 : 0;
        Vec2 e = poly.m_vertices[vertIndex2].sub(poly.m_vertices[vertIndex1]);
        float length = e.normalize();

        // If the edge length is zero ...
        if (length < Settings.EPSILON) {
            Vec2 d = xLocal.sub(poly.m_vertices[vertIndex1]);
            float dist = d.normalize();
            if (dist > radius) {
                return;
            }

            manifold.pointCount = 1;
            manifold.normal = poly.m_R.mul(d);
            manifold.points[0].id.features.incidentEdge = Collision.NULL_FEATURE;
            manifold.points[0].id.features.incidentVertex = vertIndex1;
            manifold.points[0].id.features.referenceFace = Collision.NULL_FEATURE;
            manifold.points[0].id.features.flip = 0;
            manifold.points[0].position = circle.m_position.sub(manifold.normal
                    .mul(radius));
            manifold.points[0].separation = dist - radius;
            return;
        }

        // Project the center onto the edge.
        float u = Vec2.dot(xLocal.sub(poly.m_vertices[vertIndex1]), e);
        manifold.points[0].id.features.incidentEdge = Collision.NULL_FEATURE;
        manifold.points[0].id.features.incidentVertex = Collision.NULL_FEATURE;
        manifold.points[0].id.features.referenceFace = Collision.NULL_FEATURE;
        manifold.points[0].id.features.flip = 0;
        Vec2 p = new Vec2();
        if (u <= 0.0f) {
            p.set(poly.m_vertices[vertIndex1]);
            manifold.points[0].id.features.incidentVertex = vertIndex1;
        }
        else if (u >= length) {
            p.set(poly.m_vertices[vertIndex2]);
            manifold.points[0].id.features.incidentVertex = vertIndex2;
        }
        else {
            p.set(poly.m_vertices[vertIndex1]);
            p.x += u * e.x;
            p.y += u * e.y;
            manifold.points[0].id.features.incidentEdge = vertIndex1;
        }

        Vec2 d = xLocal.sub(p);
        float dist = d.normalize();
        if (dist > radius) {
            return;
        }

        manifold.pointCount = 1;
        manifold.normal = poly.m_R.mul(d);
        // manifold.points[0].id.key = 0;
        manifold.points[0].position = circle.m_position.sub(manifold.normal
                .mul(radius));
        manifold.points[0].separation = dist - radius;
    }
}
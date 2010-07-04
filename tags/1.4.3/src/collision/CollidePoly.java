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

//Updated to rev 55 of b2CollidePoly.cpp 
package collision;

import common.Settings;
import common.Vec2;

public class CollidePoly {
    static class ClipVertex {
        Vec2 v;

        ContactID id;

        public ClipVertex() {
            v = new Vec2();
            id = new ContactID();
        }
    }

    static int clipSegmentToLine(ClipVertex vOut[], ClipVertex vIn[],
            Vec2 normal, float offset) {
        // Start with no output points
        int numOut = 0;

        // Calculate the distance of end points to the line
        float distance0 = Vec2.dot(normal, vIn[0].v) - offset;
        float distance1 = Vec2.dot(normal, vIn[1].v) - offset;

        // If the points are behind the plane
        if (distance0 <= 0.0f) {
            vOut[numOut++] = vIn[0];
        }
        if (distance1 <= 0.0f) {
            vOut[numOut++] = vIn[1];
        }

        // If the points are on different sides of the plane
        if (distance0 * distance1 < 0.0f) {
            // Find intersection point of edge and plane
            float interp = distance0 / (distance0 - distance1);
            vOut[numOut] = new ClipVertex();
            vOut[numOut].v.x = vIn[0].v.x + interp * (vIn[1].v.x - vIn[0].v.x);
            vOut[numOut].v.y = vIn[0].v.y + interp * (vIn[1].v.y - vIn[0].v.y);

            if (distance0 > 0.0f) {
                vOut[numOut].id = vIn[0].id;
            }
            else {
                vOut[numOut].id = vIn[1].id;
            }
            ++numOut;
        }

        return numOut;
    }

    static float edgeSeparation(PolyShape poly1, int edge1, PolyShape poly2) {
        
        Vec2[] vert1s = poly1.m_vertices;
        int count2 = poly2.m_vertexCount;
        Vec2[] vert2s = poly2.m_vertices;

     // Convert normal from into poly2's frame.
        assert(edge1 < poly1.m_vertexCount);
        Vec2 normal = poly1.m_R.mul(poly1.m_normals[edge1]);
        Vec2 normalLocal2 = poly2.m_R.mulT(normal);

        // Find support vertex on poly2 for -normal.
        int vertexIndex2 = 0;
        float minDot = Float.MAX_VALUE;
        for (int i = 0; i < count2; ++i) {
            float dot = Vec2.dot(vert2s[i], normalLocal2);
            if (dot < minDot) {
                minDot = dot;
                vertexIndex2 = i;
            }
        }

        Vec2 v1 = poly1.m_R.mul(vert1s[edge1]).addLocal(poly1.m_position);
        Vec2 v2 = poly2.m_R.mul(vert2s[vertexIndex2]).addLocal(poly2.m_position);
        float separation = Vec2.dot(v2.subLocal(v1), normal);
        return separation;
    }

    // Find the max separation between poly1 and poly2 using face normals
    // from poly1.
    static MaxSeparation findMaxSeparation(PolyShape poly1, PolyShape poly2, boolean conservative) {
        MaxSeparation separation = new MaxSeparation();

        int count1 = poly1.m_vertexCount;
        
        // Vector pointing from the origin of poly1 to the origin of poly2.
        Vec2 d = poly2.m_position.sub(poly1.m_position);
        Vec2 dLocal1 = poly1.m_R.mulT(d);

        // Get support vertex as a hint for our search
        int edge = 0;
        float maxDot = -Float.MAX_VALUE;
        for (int i = 0; i < count1; ++i) {
            float dot = Vec2.dot(poly1.m_normals[i], dLocal1);
            if (dot > maxDot) {
                maxDot = dot;
                edge = i;
            }
        }

        // Get the separation for the edge normal.
        float s = edgeSeparation(poly1, edge, poly2);
        if (s > 0.0f && conservative == false){
            separation.bestSeparation = s;
            return separation;
        }

        // Check the separation for the neighboring edges.
        int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
        float sPrev = edgeSeparation(poly1, prevEdge, poly2);
        if (sPrev > 0.0f && conservative == false) {
            separation.bestSeparation = sPrev;
            return separation;
        }

        int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
        float sNext = edgeSeparation(poly1, nextEdge, poly2);
        if (sNext > 0.0f && conservative == false){
            separation.bestSeparation = sNext;
            return separation;
        }

        // Find the best edge and the search direction.
        int bestEdge;
        float bestSeparation;
        int increment;
        if (sPrev > s && sPrev > sNext) {
            increment = -1;
            bestEdge = prevEdge;
            bestSeparation = sPrev;
        }
        else if (sNext > s){
            increment = 1;
            bestEdge = nextEdge;
            bestSeparation = sNext;
        } else {
            separation.bestFaceIndex = edge;
            separation.bestSeparation = s;
            return separation;
        }

        while (true) {
            if (increment == -1)
                edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
            else
                edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;

            s = edgeSeparation(poly1, edge, poly2);
            if (s > 0.0f && conservative == false) {
                separation.bestSeparation = s;
                return separation;
            }

            if (s > bestSeparation){
                bestEdge = edge;
                bestSeparation = s;
            } else {
                break;
            }
        }

        separation.bestFaceIndex = bestEdge;
        separation.bestSeparation = bestSeparation;

        return separation;
    }

    static void findIncidentEdge(ClipVertex c[], PolyShape poly1, int edge1,
            PolyShape poly2) {
        int count1 = poly1.m_vertexCount;
        Vec2[] vert1s = poly1.m_vertices;
        int count2 = poly2.m_vertexCount;
        Vec2[] vert2s = poly2.m_vertices;

        // Get the vertices associated with edge1.
        int vertex11 = edge1;
        int vertex12 = edge1 + 1 == count1 ? 0 : edge1 + 1;

        // Get the normal of edge1.
        Vec2 normal1Local1 = Vec2.cross(vert1s[vertex12].sub(vert1s[vertex11]),
                1.0f);
        normal1Local1.normalize();
        Vec2 normal1 = poly1.m_R.mul(normal1Local1);
        Vec2 normal1Local2 = poly2.m_R.mulT(normal1);

        // Find the incident edge on poly2.
        int vertex21 = 0, vertex22 = 0;
        float minDot = Float.MAX_VALUE;
        for (int i = 0; i < count2; ++i) {
            int i1 = i;
            int i2 = i + 1 < count2 ? i + 1 : 0;

            Vec2 normal2Local2 = Vec2.cross(vert2s[i2].sub(vert2s[i1]), 1.0f);
            normal2Local2.normalize();
            float dot = Vec2.dot(normal2Local2, normal1Local2);
            if (dot < minDot) {
                minDot = dot;
                vertex21 = i1;
                vertex22 = i2;
            }
        }

        // Build the clip vertices for the incident edge.
        c[0] = new ClipVertex();
        c[1] = new ClipVertex();

        c[0].v = poly2.m_R.mul(vert2s[vertex21]).addLocal(poly2.m_position);
        c[0].id.features.referenceFace = edge1;
        c[0].id.features.incidentEdge = vertex21;
        c[0].id.features.incidentVertex = vertex21;

        c[1].v = poly2.m_R.mul(vert2s[vertex22]).addLocal(poly2.m_position);
        c[1].id.features.referenceFace = edge1;
        c[1].id.features.incidentEdge = vertex21;
        c[1].id.features.incidentVertex = vertex22;
    }

    // Find edge normal of max separation on A - return if separating axis is
    // found
    // Find edge normal of max separation on B - return if separation axis is
    // found
    // Choose reference edge as min(minA, minB)
    // Find incident edge
    // Clip

    // The normal points from 1 to 2
    public static void collidePoly(Manifold manif, PolyShape polyA,
            PolyShape polyB, boolean conservative) {
        // ~84 vec2 creations in Pyramid test, per run
        // (Some from called functions)
        // Runs ~625 times per step
        // 625 * 84 = 52,500, out of ~95,000 total creations
        // TODO Probably worth optimizing...
        // Runs ~1000 times per step in DominoTower test
        //testbed.PTest.debugCount++;
        manif.pointCount = 0; // Fixed a problem with contacts
        MaxSeparation sepA = findMaxSeparation(polyA, polyB, conservative);
        if (sepA.bestSeparation > 0.0f && conservative == false) {
            return;
        }

        MaxSeparation sepB = findMaxSeparation(polyB, polyA, conservative);
        if (sepB.bestSeparation > 0.0f && conservative == false) {
            return;
        }

        PolyShape poly1; // reference poly
        PolyShape poly2; // incident poly
        int edge1; // reference edge
        byte flip;
        float k_relativeTol = 0.98f;
        float k_absoluteTol = 0.001f;

        // TODO_ERIN use "radius" of poly for absolute tolerance.
        if (sepB.bestSeparation > k_relativeTol * sepA.bestSeparation
                + k_absoluteTol) {
            poly1 = polyB;
            poly2 = polyA;
            edge1 = sepB.bestFaceIndex;
            flip = 1;
        }
        else {
            poly1 = polyA;
            poly2 = polyB;
            edge1 = sepA.bestFaceIndex;
            flip = 0;
        }

        ClipVertex incidentEdge[] = new ClipVertex[2];
        findIncidentEdge(incidentEdge, poly1, edge1, poly2);

        int count1 = poly1.m_vertexCount;

        Vec2[] vert1s = poly1.m_vertices;

        Vec2 v11 = vert1s[edge1];
        Vec2 v12 = edge1 + 1 < count1 ? vert1s[edge1 + 1] : vert1s[0];

        Vec2 sideNormal = poly1.m_R.mul(v12.sub(v11));
        sideNormal.normalize();
        Vec2 frontNormal = Vec2.cross(sideNormal, 1.0f);

        v11 = poly1.m_R.mul(v11).addLocal(poly1.m_position);
        v12 = poly1.m_R.mul(v12).addLocal(poly1.m_position);

        float frontOffset = Vec2.dot(frontNormal, v11);
        float sideOffset1 = -Vec2.dot(sideNormal, v11);
        float sideOffset2 = Vec2.dot(sideNormal, v12);

        // Clip incident edge against extruded edge1 side edges.
        ClipVertex clipPoints1[] = new ClipVertex[2];
        ClipVertex clipPoints2[] = new ClipVertex[2];
        int np;

        // Clip to box side 1
        np = clipSegmentToLine(clipPoints1, incidentEdge, sideNormal.negate(),
                sideOffset1);

        if (np < 2) {
            return;
        }

        // Clip to negative box side 1
        np = clipSegmentToLine(clipPoints2, clipPoints1, sideNormal,
                sideOffset2);

        if (np < 2) {
            return;
        }

        // Now clipPoints2 contains the clipped points.
        manif.normal = flip != 0 ? frontNormal.negate() : frontNormal.clone();

        int pointCount = 0;
        for (int i = 0; i < Settings.maxManifoldPoints; ++i) {
            float separation = Vec2.dot(frontNormal, clipPoints2[i].v)
                    - frontOffset;

            if (separation <= 0.0f && conservative == false) {
                ContactPoint cp = manif.points[pointCount];
                cp.separation = separation;
                cp.position = clipPoints2[i].v.clone();
                cp.id = new ContactID(clipPoints2[i].id);
                cp.id.features.flip = flip;
                ++pointCount;
            }
        }

        manif.pointCount = pointCount;

        return;
    }
}

class MaxSeparation {
    public int bestFaceIndex;

    public float bestSeparation;
}
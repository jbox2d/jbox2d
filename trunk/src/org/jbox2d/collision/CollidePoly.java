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

//Updated to rev 55->108 of b2CollidePoly.cpp 

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
            vOut[numOut] = new ClipVertex();
            vOut[numOut].id = new ContactID(vIn[0].id);
            vOut[numOut++].v = vIn[0].v.clone();
        }
        if (distance1 <= 0.0f) {
            vOut[numOut] = new ClipVertex();
            vOut[numOut].id = new ContactID(vIn[1].id);
            vOut[numOut++].v = vIn[1].v.clone();
        }

        // If the points are on different sides of the plane
        if (distance0 * distance1 < 0.0f) {
            // Find intersection point of edge and plane
            float interp = distance0 / (distance0 - distance1);
            vOut[numOut] = new ClipVertex();
            vOut[numOut].v.x = vIn[0].v.x + interp * (vIn[1].v.x - vIn[0].v.x);
            vOut[numOut].v.y = vIn[0].v.y + interp * (vIn[1].v.y - vIn[0].v.y);

            if (distance0 > 0.0f) {
                vOut[numOut].id = new ContactID(vIn[0].id);
            }
            else {
                vOut[numOut].id = new ContactID(vIn[1].id);
            }
            ++numOut;
        }

        return numOut;
    }

    static float edgeSeparation(PolygonShape poly1, XForm xf1,
    							int edge1, 
    							PolygonShape poly2, XForm xf2) {
        
        assert(0 <= edge1 && edge1 < poly1.m_vertexCount);

    	// Convert normal from poly1's frame into poly2's frame.
    	Vec2 normal1World = Mat22.mul(xf1.R, poly1.m_normals[edge1]);
    	Vec2 normal1 = Mat22.mulT(xf2.R, normal1World);

        // Find support vertex on poly2 for -normal.
        int index = 0;
        float minDot = Float.MAX_VALUE;
        for (int i = 0; i < poly2.m_vertexCount; ++i) {
            float dot = Vec2.dot(poly2.m_vertices[i], normal1);
            if (dot < minDot) {
                minDot = dot;
                index = i;
            }
        }

    	Vec2 v1 = XForm.mul(xf1, poly1.m_vertices[edge1]);
    	Vec2 v2 = XForm.mul(xf2, poly2.m_vertices[index]);
    	float separation = Vec2.dot(v2.sub(v1), normal1World);
        
        return separation;
    }

    // Find the max separation between poly1 and poly2 using face normals
    // from poly1.
    static MaxSeparation findMaxSeparation(PolygonShape poly1, XForm xf1,
    									   PolygonShape poly2, XForm xf2) {
        MaxSeparation separation = new MaxSeparation();

        int count1 = poly1.m_vertexCount;
        
        // Vector pointing from the centroid of poly1 to the centroid of poly2.
    	Vec2 d = XForm.mul(xf2, poly2.m_centroid).subLocal(XForm.mul(xf1, poly1.m_centroid));
    	Vec2 dLocal1 = Mat22.mulT(xf1.R, d);

    	// Find edge normal on poly1 that has the largest projection onto d.
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
        float s = edgeSeparation(poly1, xf1, edge, poly2, xf2);
        if (s > 0.0f){
            separation.bestSeparation = s;
            return separation;
        }

        // Check the separation for the previous edge normal.
        int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
        float sPrev = edgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
        if (sPrev > 0.0f) {
            separation.bestSeparation = sPrev;
            return separation;
        }

        int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
        float sNext = edgeSeparation(poly1, xf1, nextEdge, poly2, xf2);
        if (sNext > 0.0f){
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

        // Perform a local search for the best edge normal.
        while (true) {
            if (increment == -1)
                edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
            else
                edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;

            s = edgeSeparation(poly1, xf1, edge, poly2, xf2);
            if (s > 0.0f) {
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

    static void findIncidentEdge(ClipVertex c[], 
    							 PolygonShape poly1, XForm xf1, int edge1,
    							 PolygonShape poly2, XForm xf2) {
    	
    	assert(0 <= edge1 && edge1 < poly1.m_vertexCount);

    	// Get the normal of the reference edge in poly2's frame.
    	Vec2 normal1 = Mat22.mulT(xf2.R, Mat22.mul(xf1.R, poly1.m_normals[edge1]));

    	// Find the incident edge on poly2.
    	int index = 0;
    	float minDot = Float.MAX_VALUE;
    	for (int i = 0; i < poly2.m_vertexCount; ++i) {
    		float dot = Vec2.dot(normal1, poly2.m_normals[i]);
    		if (dot < minDot) {
    			minDot = dot;
    			index = i;
    		}
    	}

    	// Build the clip vertices for the incident edge.
    	int i1 = index;
    	int i2 = i1 + 1 < poly2.m_vertexCount ? i1 + 1 : 0;
    	
    	c[0] = new ClipVertex();
    	c[1] = new ClipVertex();

    	c[0].v = XForm.mul(xf2, poly2.m_vertices[i1]);
    	c[0].id.features.referenceFace = edge1;
    	c[0].id.features.incidentEdge = i1;
    	c[0].id.features.incidentVertex = 0;
    	
    	

    	c[1].v = XForm.mul(xf2, poly2.m_vertices[i2]);
    	c[1].id.features.referenceFace = edge1;
    	c[1].id.features.incidentEdge = i2;
    	c[1].id.features.incidentVertex = 1;
    	
    }

    // Find edge normal of max separation on A - return if separating axis is
    // found
    // Find edge normal of max separation on B - return if separation axis is
    // found
    // Choose reference edge as min(minA, minB)
    // Find incident edge
    // Clip

    // The normal points from 1 to 2
    public static void collidePoly(Manifold manif, 
    		PolygonShape polyA, XForm xfA,
            PolygonShape polyB, XForm xfB) {

        //testbed.PTest.debugCount++;
        manif.pointCount = 0; // Fixed a problem with contacts
        MaxSeparation sepA = findMaxSeparation(polyA, xfA, polyB, xfB);
        if (sepA.bestSeparation > 0.0f) {
            return;
        }

        MaxSeparation sepB = findMaxSeparation(polyB, xfB, polyA, xfA);
        if (sepB.bestSeparation > 0.0f) {
            return;
        }

        PolygonShape poly1; // reference poly
        PolygonShape poly2; // incident poly
        XForm xf1 = new XForm();
        XForm xf2 = new XForm();
        int edge1; // reference edge
        byte flip;
        float k_relativeTol = 0.98f;
        float k_absoluteTol = 0.001f;

        // TODO_ERIN use "radius" of poly for absolute tolerance.
        if (sepB.bestSeparation > k_relativeTol * sepA.bestSeparation
                + k_absoluteTol) {
            poly1 = polyB;
            poly2 = polyA;
            xf1.set(xfB);
    		xf2.set(xfA);
            edge1 = sepB.bestFaceIndex;
            flip = 1;
        }
        else {
            poly1 = polyA;
            poly2 = polyB;
            xf1.set(xfA);
    		xf2.set(xfB);
            edge1 = sepA.bestFaceIndex;
            flip = 0;
        }

        ClipVertex incidentEdge[] = new ClipVertex[2];
        findIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

        int count1 = poly1.m_vertexCount;
        Vec2[] vert1s = poly1.m_vertices;

        Vec2 v11 = vert1s[edge1];
        Vec2 v12 = edge1 + 1 < count1 ? vert1s[edge1 + 1] : vert1s[0];

        Vec2 sideNormal = Mat22.mul(xf1.R, v12.sub(v11));
        sideNormal.normalize();
        Vec2 frontNormal = Vec2.cross(sideNormal, 1.0f);

        v11 = XForm.mul(xf1, v11);
    	v12 = XForm.mul(xf1, v12);

        float frontOffset = Vec2.dot(frontNormal, v11);
        float sideOffset1 = -Vec2.dot(sideNormal, v11);
        float sideOffset2 = Vec2.dot(sideNormal, v12);

        // Clip incident edge against extruded edge1 side edges.
        ClipVertex clipPoints1[] = new ClipVertex[2];
        ClipVertex clipPoints2[] = new ClipVertex[2];
        int np;

        // Clip to box side 1
        np = clipSegmentToLine(clipPoints1, incidentEdge, sideNormal.negate(), sideOffset1);

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
        manif.normal = (flip != 0) ? frontNormal.negate() : frontNormal.clone();

        int pointCount = 0;
        for (int i = 0; i < Settings.maxManifoldPoints; ++i) {
            float separation = Vec2.dot(frontNormal, clipPoints2[i].v)
                    - frontOffset;

            if (separation <= 0.0f) {
                ManifoldPoint cp = manif.points[pointCount];
                cp.separation = separation;
                cp.localPoint1 = XForm.mulT(xfA, clipPoints2[i].v);
    			cp.localPoint2 = XForm.mulT(xfB, clipPoints2[i].v);
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
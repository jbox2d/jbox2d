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

//Updated to rev 55->108->139 of b2CollidePoly.cpp 

/** Polygon overlap solver - for internal use. */
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
        
        int count1 = poly1.getVertexCount();
    	Vec2[] vertices1 = poly1.getVertices();
    	Vec2[] normals1 = poly1.getNormals();

    	int count2 = poly2.getVertexCount();
    	Vec2[] vertices2 = poly2.getVertices();

    	assert(0 <= edge1 && edge1 < count1);

    	// Convert normal from poly1's frame into poly2's frame.
    	Vec2 normal1World = Mat22.mul(xf1.R, normals1[edge1]);
    	float normal1x = Vec2.dot(normal1World, xf2.R.col1);
    	float normal1y = Vec2.dot(normal1World, xf2.R.col2);

        // Find support vertex on poly2 for -normal.
        int index = 0;
        float minDot = Float.MAX_VALUE;
        for (int i = 0; i < count2; ++i) {
            float dot = vertices2[i].x * normal1x + vertices2[i].y * normal1y; 
            	//Vec2.dot(poly2.m_vertices[i], normal1);
            if (dot < minDot) {
                minDot = dot;
                index = i;
            }
        }

    	//Vec2 v1 = XForm.mul(xf1, poly1.m_vertices[edge1]);
        Vec2 v = vertices1[edge1];
        float v1x = xf1.position.x + xf1.R.col1.x * v.x + xf1.R.col2.x * v.y;
		float v1y = xf1.position.y + xf1.R.col1.y * v.x + xf1.R.col2.y * v.y;
		Vec2 v3 = vertices2[index];
		//Vec2 v2 = XForm.mul(xf2, poly2.m_vertices[index]);
    	float v2x = xf2.position.x + xf2.R.col1.x * v3.x + xf2.R.col2.x * v3.y;
    	float v2y = xf2.position.y + xf2.R.col1.y * v3.x + xf2.R.col2.y * v3.y;
		//float separation = Vec2.dot(v2.sub(v1), normal1World);
    	float separation = (v2x-v1x) * normal1World.x + (v2y-v1y) * normal1World.y;
        
        return separation;
    }

    // Find the max separation between poly1 and poly2 using face normals
    // from poly1.
    static MaxSeparation findMaxSeparation(PolygonShape poly1, XForm xf1,
    									   PolygonShape poly2, XForm xf2) {
        MaxSeparation separation = new MaxSeparation();

        int count1 = poly1.getVertexCount();
        Vec2[] normals1 = poly1.getNormals();
        
		Vec2 v = poly1.getCentroid();
		Vec2 v1 = poly2.getCentroid();
        
        // Vector pointing from the centroid of poly1 to the centroid of poly2.
        //Vec2 d = XForm.mul(xf2, poly2.m_centroid).subLocal(XForm.mul(xf1, poly1.m_centroid));
    	//Vec2 dLocal1 = Mat22.mulT(xf1.R, d);
    	float dx = xf2.position.x + xf2.R.col1.x * v1.x + xf2.R.col2.x * v1.y
    			 - (xf1.position.x + xf1.R.col1.x * v.x + xf1.R.col2.x * v.y);
    	float dy = xf2.position.y + xf2.R.col1.y * v1.x + xf2.R.col2.y * v1.y
    			 - (xf1.position.y + xf1.R.col1.y * v.x + xf1.R.col2.y * v.y);
		Vec2 b = xf1.R.col1;
		Vec2 b1 = xf1.R.col2;
    	Vec2 dLocal1 = new Vec2((dx * b.x + dy * b.y), (dx * b1.x + dy * b1.y));

    	// Find edge normal on poly1 that has the largest projection onto d.
        int edge = 0;
        float maxDot = -Float.MAX_VALUE;
        for (int i = 0; i < count1; ++i) {
            float dot = Vec2.dot(normals1[i], dLocal1);
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
    	
    	int count1 = poly1.getVertexCount();
    	Vec2[] normals1 = poly1.getNormals();

    	int count2 = poly2.getVertexCount();
    	Vec2[] vertices2 = poly2.getVertices();
    	Vec2[] normals2 = poly2.getNormals();

    	assert(0 <= edge1 && edge1 < count1);

    	// Get the normal of the reference edge in poly2's frame.
    	Vec2 normal1 = Mat22.mulT(xf2.R, Mat22.mul(xf1.R, normals1[edge1]));

    	// Find the incident edge on poly2.
    	int index = 0;
    	float minDot = Float.MAX_VALUE;
    	for (int i = 0; i < count2; ++i) {
    		float dot = Vec2.dot(normal1, normals2[i]);
    		if (dot < minDot) {
    			minDot = dot;
    			index = i;
    		}
    	}

    	// Build the clip vertices for the incident edge.
    	int i1 = index;
    	int i2 = i1 + 1 < count2 ? i1 + 1 : 0;
    	
    	c[0] = new ClipVertex();
    	c[1] = new ClipVertex();

    	c[0].v = XForm.mul(xf2, vertices2[i1]);
    	c[0].id.features.referenceEdge = edge1;
    	c[0].id.features.incidentEdge = i1;
    	c[0].id.features.incidentVertex = 0;
    	
    	

    	c[1].v = XForm.mul(xf2, vertices2[i2]);
    	c[1].id.features.referenceEdge = edge1;
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
    public static void collidePolygons(Manifold manif, 
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

        int count1 = poly1.getVertexCount();
        Vec2[] vertices1 = poly1.getVertices();

        Vec2 v11 = vertices1[edge1];
        Vec2 v12 = edge1 + 1 < count1 ? vertices1[edge1 + 1] : vertices1[0];

        Vec2 sideNormal = Mat22.mul(xf1.R, v12.sub(v11));
        sideNormal.normalize();
        Vec2 frontNormal = Vec2.cross(sideNormal, 1.0f);

        //v11 = XForm.mul(xf1, v11);
    	//v12 = XForm.mul(xf1, v12);
    	float v11x = xf1.position.x + xf1.R.col1.x * v11.x + xf1.R.col2.x * v11.y;
    	float v11y = xf1.position.y + xf1.R.col1.y * v11.x + xf1.R.col2.y * v11.y;
    	float v12x = xf1.position.x + xf1.R.col1.x * v12.x + xf1.R.col2.x * v12.y; 
		float v12y = xf1.position.y + xf1.R.col1.y * v12.x + xf1.R.col2.y * v12.y;

        float frontOffset = frontNormal.x * v11x + frontNormal.y * v11y;
        float sideOffset1 = -(sideNormal.x * v11x + sideNormal.y * v11y);
        float sideOffset2 = sideNormal.x * v12x + sideNormal.y * v12y;

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
                //cp.localPoint1 = XForm.mulT(xfA, clipPoints2[i].v);
    			//cp.localPoint2 = XForm.mulT(xfB, clipPoints2[i].v);
                Vec2 vec = clipPoints2[i].v;
                float v1x = vec.x-xfA.position.x;
				float v1y = vec.y-xfA.position.y;
    			cp.localPoint1.x = (v1x * xfA.R.col1.x + v1y * xfA.R.col1.y);
    			cp.localPoint1.y = (v1x * xfA.R.col2.x + v1y * xfA.R.col2.y);
    			
				v1x = vec.x-xfB.position.x;
				v1y = vec.y-xfB.position.y;
				cp.localPoint2.x = (v1x * xfB.R.col1.x + v1y * xfB.R.col1.y);
    			cp.localPoint2.y = (v1x * xfB.R.col2.x + v1y * xfB.R.col2.y);
    			
    			cp.id = new ContactID(clipPoints2[i].id);
                cp.id.features.flip = flip;
                ++pointCount;
            }
        }

        manif.pointCount = pointCount;

        return;
    }
}

/** Holder class used internally in CollidePoly. */
class MaxSeparation {
    public int bestFaceIndex;
    public float bestSeparation;
}
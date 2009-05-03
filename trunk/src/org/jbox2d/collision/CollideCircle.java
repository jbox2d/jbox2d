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

//Updated to rev 139 of b2CollideCircle.cpp

/**
 * Circle/circle and circle/polygon overlap solver -
 * for internal use only.
 */
public class CollideCircle {

    public static void collideCircles(Manifold manifold, 
    		CircleShape circle1, XForm xf1,
            CircleShape circle2, XForm xf2) {
        manifold.pointCount = 0;

    	Vec2 p1 = XForm.mul(xf1, circle1.getLocalPosition());
    	Vec2 p2 = XForm.mul(xf2, circle2.getLocalPosition());

    	Vec2 d = p2.sub(p1);
        
    	float distSqr = Vec2.dot(d, d);

    	float r1 = circle1.getRadius();
    	float r2 = circle2.getRadius();
    	float radiusSum = r1+r2;
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

    	p1.addLocal(manifold.normal.mul(r1));
    	p2.subLocal(manifold.normal.mul(r2));

    	Vec2 p = new Vec2(0.5f * (p1.x + p2.x), 0.5f * (p1.y + p2.y));

    	manifold.points[0].localPoint1 = XForm.mulT(xf1, p);
    	manifold.points[0].localPoint2 = XForm.mulT(xf2, p);

    }
    
    public static void collidePointAndCircle(Manifold manifold, 
    		PointShape point1, XForm xf1,
            CircleShape circle2, XForm xf2) {
        manifold.pointCount = 0;

    	Vec2 p1 = XForm.mul(xf1, point1.getLocalPosition());
    	Vec2 p2 = XForm.mul(xf2, circle2.getLocalPosition());

    	Vec2 d = p2.sub(p1);
        
    	float distSqr = Vec2.dot(d, d);

    	float r2 = circle2.getRadius();

        if (distSqr > r2*r2) {
            return;
        }

        float separation;
        if (distSqr < Settings.EPSILON) {
            separation = -r2;
            manifold.normal.set(0.0f, 1.0f);
        }
        else {
            float dist = (float) Math.sqrt(distSqr);
            separation = dist - r2;
            float a = 1.0f / dist;
            manifold.normal.x = a * d.x;
            manifold.normal.y = a * d.y;
        }

        manifold.pointCount = 1;
        //manifold.points[0].id.key = 0;
        manifold.points[0].id.zero(); //use this instead of zeroing through key
        manifold.points[0].separation = separation;

    	p2.subLocal(manifold.normal.mul(r2));

    	Vec2 p = new Vec2(0.5f * (p1.x + p2.x), 0.5f * (p1.y + p2.y));

    	manifold.points[0].localPoint1 = XForm.mulT(xf1, p);
    	manifold.points[0].localPoint2 = XForm.mulT(xf2, p);

    }

    public static void collidePolygonAndCircle(Manifold manifold, 
    		PolygonShape polygon, XForm xf1,
            CircleShape circle, XForm xf2) {
    	
        manifold.pointCount = 0;

        // Compute circle position in the frame of the polygon.
        // INLINED
        //Vec2 c = XForm.mul(xf2, circle.getLocalPosition());
    	//Vec2 cLocal = XForm.mulT(xf1, c);
        
		float cx = xf2.position.x + xf2.R.col1.x * circle.m_localPosition.x + xf2.R.col2.x * circle.m_localPosition.y;
		float cy = xf2.position.y + xf2.R.col1.y * circle.m_localPosition.x + xf2.R.col2.y * circle.m_localPosition.y;
		float v1x = cx - xf1.position.x;
		float v1y = cy - xf1.position.y;
		float cLocalx = v1x * xf1.R.col1.x + v1y * xf1.R.col1.y;
		float cLocaly = v1x * xf1.R.col2.x + v1y * xf1.R.col2.y;
    	
        // Find edge with maximum separation.
        int normalIndex = 0;
        float separation = -Float.MAX_VALUE;
        float radius = circle.getRadius();
        int vertexCount = polygon.getVertexCount();
        Vec2[] vertices = polygon.getVertices();
        Vec2[] normals = polygon.getNormals();
        for (int i = 0; i < vertexCount; ++i) {

        	// INLINED
            //float s = Vec2.dot(normals[i], cLocal.sub(vertices[i]));
            float s = normals[i].x * (cLocalx - vertices[i].x) + normals[i].y * (cLocaly - vertices[i].y);
            
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
			// INLINED
            //manifold.normal = Mat22.mul(xf1.R, normals[normalIndex]);
            manifold.normal.set(xf1.R.col1.x * normals[normalIndex].x + xf1.R.col2.x * normals[normalIndex].y, xf1.R.col1.y * normals[normalIndex].x + xf1.R.col2.y * normals[normalIndex].y);
            
            manifold.points[0].id.features.incidentEdge = normalIndex;
            manifold.points[0].id.features.incidentVertex = Collision.NULL_FEATURE;
            manifold.points[0].id.features.referenceEdge = 0;
            manifold.points[0].id.features.flip = 0;
            // INLINED
    		//Vec2 position = c.sub(manifold.normal.mul(radius));
    		//manifold.points[0].localPoint1 = XForm.mulT(xf1, position);
    		//manifold.points[0].localPoint2 = XForm.mulT(xf2, position);
            
            float positionx = cx - manifold.normal.x * radius;
            float positiony = cy - manifold.normal.y * radius;
			float v1x1 = positionx-xf1.position.x;
			float v1y1 = positiony-xf1.position.y;
			manifold.points[0].localPoint1.set((v1x1 * xf1.R.col1.x + v1y1 * xf1.R.col1.y), (v1x1 * xf1.R.col2.x + v1y1 * xf1.R.col2.y));
			float v1x2 = positionx-xf2.position.x;
			float v1y2 = positiony-xf2.position.y;
			manifold.points[0].localPoint2.set((v1x2 * xf2.R.col1.x + v1y2 * xf2.R.col1.y), (v1x2 * xf2.R.col2.x + v1y2 * xf2.R.col2.y));
			
            manifold.points[0].separation = separation - radius;
            return;
        }

        // Project the circle center onto the edge segment.
        int vertIndex1 = normalIndex;
        int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
        // INLINED
        //Vec2 e = vertices[vertIndex2].sub(vertices[vertIndex1]);
        //float length = e.normalize();
        
        float ex = vertices[vertIndex2].x - vertices[vertIndex1].x;
        float ey = vertices[vertIndex2].y - vertices[vertIndex1].y;
        float length = (float) Math.sqrt(ex * ex + ey * ey);
        assert(length > Settings.EPSILON);
        float invLength = 1.0f / length;
        ex *= invLength;
        ey *= invLength;

        // Project the center onto the edge.
        // INLINED
        //float u = Vec2.dot(cLocal.sub(vertices[vertIndex1]), e);
        float u = (cLocalx - vertices[vertIndex1].x) * ex + (cLocaly - vertices[vertIndex1].y) * ey;
        
        float px;
        float py;
        if (u <= 0.0f) {
        	px = vertices[vertIndex1].x;
        	py = vertices[vertIndex1].y;
            manifold.points[0].id.features.incidentEdge = Collision.NULL_FEATURE;
            manifold.points[0].id.features.incidentVertex = vertIndex1;
        }
        else if (u >= length) {
        	px = vertices[vertIndex2].x;
        	py = vertices[vertIndex2].y;
            manifold.points[0].id.features.incidentEdge = Collision.NULL_FEATURE;
            manifold.points[0].id.features.incidentVertex = vertIndex2;
        }
        else {
        	px = vertices[vertIndex1].x;
        	py = vertices[vertIndex1].y;
            px += u * ex;
            py += u * ey;
            manifold.points[0].id.features.incidentEdge = normalIndex;
            manifold.points[0].id.features.incidentVertex = Collision.NULL_FEATURE;
        }
        
        // INLINED
        //Vec2 d = cLocal.sub(p);
        //float dist = d.normalize();
        
        float dx = cLocalx - px;
        float dy = cLocaly - py;
        float dist = (float) Math.sqrt(dx * dx + dy * dy);
        if (dist > radius) {
            return;
        }
        if (dist >= Settings.EPSILON) {
        	float invDist = 1.0f / dist;
        	dx *= invDist;
        	dy *= invDist;
        }
        
        manifold.pointCount = 1;
        
        // INLINED
    	//manifold.normal = Mat22.mul(xf1.R, d);
    	//Vec2 position = c.sub(manifold.normal.mul(radius));
    	//manifold.points[0].localPoint1 = XForm.mulT(xf1, position);
    	//manifold.points[0].localPoint2 = XForm.mulT(xf2, position);
        
    	manifold.normal.set(xf1.R.col1.x * dx + xf1.R.col2.x * dy, xf1.R.col1.y * dx + xf1.R.col2.y * dy);
    	float positionx = cx - manifold.normal.x * radius;
    	float positiony = cy - manifold.normal.y * radius;
		float v1x1 = positionx - xf1.position.x;
		float v1y1 = positiony - xf1.position.y;
		manifold.points[0].localPoint1.set((v1x1 * xf1.R.col1.x + v1y1 * xf1.R.col1.y), (v1x1 * xf1.R.col2.x + v1y1 * xf1.R.col2.y));
		float v1x2 = positionx - xf2.position.x;
		float v1y2 = positiony - xf2.position.y;
		manifold.points[0].localPoint2.set((v1x2 * xf2.R.col1.x + v1y2 * xf2.R.col1.y), (v1x2 * xf2.R.col2.x + v1y2 * xf2.R.col2.y));
    	
    	manifold.points[0].separation = dist - radius;
        manifold.points[0].id.features.referenceEdge = 0;
        manifold.points[0].id.features.flip = 0;
    }
    
    public static void collidePolygonAndPoint(Manifold manifold, 
    		PolygonShape polygon, XForm xf1,
            PointShape point, XForm xf2) {
    	
        manifold.pointCount = 0;

        // Compute circle position in the frame of the polygon.
        Vec2 c = XForm.mul(xf2, point.getLocalPosition());
    	Vec2 cLocal = XForm.mulT(xf1, c);

        // Find edge with maximum separation.
        int normalIndex = 0;
        float separation = -Float.MAX_VALUE;
        
        int vertexCount = polygon.getVertexCount();
        Vec2[] vertices = polygon.getVertices();
        Vec2[] normals = polygon.getNormals();
        for (int i = 0; i < vertexCount; ++i) {

            float s = Vec2.dot(normals[i], cLocal.sub(vertices[i]));
            if (s > 0) {
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
            manifold.normal = Mat22.mul(xf1.R, normals[normalIndex]);
            manifold.points[0].id.features.incidentEdge = normalIndex;
            manifold.points[0].id.features.incidentVertex = Collision.NULL_FEATURE;
            manifold.points[0].id.features.referenceEdge = 0;
            manifold.points[0].id.features.flip = 0;
    		Vec2 position = c;
    		manifold.points[0].localPoint1 = XForm.mulT(xf1, position);
    		manifold.points[0].localPoint2 = XForm.mulT(xf2, position);
            manifold.points[0].separation = separation;
            return;
        }

        // Project the circle center onto the edge segment.
        int vertIndex1 = normalIndex;
        int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
        Vec2 e = vertices[vertIndex2].sub(vertices[vertIndex1]);
        float length = e.normalize();
        assert(length > Settings.EPSILON);

        // Project the center onto the edge.
        float u = Vec2.dot(cLocal.sub(vertices[vertIndex1]), e);

        Vec2 p = new Vec2();
        if (u <= 0.0f) {
            p.set(vertices[vertIndex1]);
            manifold.points[0].id.features.incidentEdge = Collision.NULL_FEATURE;
            manifold.points[0].id.features.incidentVertex = vertIndex1;
        }
        else if (u >= length) {
            p.set(vertices[vertIndex2]);
            manifold.points[0].id.features.incidentEdge = Collision.NULL_FEATURE;
            manifold.points[0].id.features.incidentVertex = vertIndex2;
        }
        else {
            p.set(vertices[vertIndex1]);
            p.x += u * e.x;
            p.y += u * e.y;
            manifold.points[0].id.features.incidentEdge = normalIndex;
            manifold.points[0].id.features.incidentVertex = Collision.NULL_FEATURE;
        }

        Vec2 d = cLocal.sub(p);
        float dist = d.normalize();
        if (dist > 0) {
            return;
        }

        manifold.pointCount = 1;
        
    	manifold.normal = Mat22.mul(xf1.R, d);
    	Vec2 position = c;
    	manifold.points[0].localPoint1 = XForm.mulT(xf1, position);
    	manifold.points[0].localPoint2 = XForm.mulT(xf2, position);
        manifold.points[0].separation = dist;
        manifold.points[0].id.features.referenceEdge = 0;
        manifold.points[0].id.features.flip = 0;
    }
}
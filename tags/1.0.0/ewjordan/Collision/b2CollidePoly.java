/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
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

//#include "b2Collision.h"
//#include "b2Shape.h"

class ClipVertex{
	public b2Vec2 v;
	public b2ContactID id;
	
	public ClipVertex(){
		v = new b2Vec2(0f,0f);
		id = new b2ContactID();
	}
	
	public ClipVertex(b2Vec2 _v, b2ContactID _id){
		v = new b2Vec2(_v);
		id = new b2ContactID(_id);
	}
	
};

//Java note: a bunch of global functions live here...

static int32 ClipSegmentToLine(ClipVertex[] vOut, ClipVertex[] vIn, b2Vec2 normal, float32 offset){
	// Start with no output points
	int32 numOut = 0;

	// Calculate the distance of end points to the line
	float32 distance0 = b2Math.b2Dot(normal, vIn[0].v) - offset;
	float32 distance1 = b2Math.b2Dot(normal, vIn[1].v) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0f) vOut[numOut++] = new ClipVertex(vIn[0]); //Java note: not sure if the new creation is necessary or not...check it out
	if (distance1 <= 0.0f) vOut[numOut++] = new ClipVertex(vIn[1]);

	// If the points are on different sides of the plane
	if (distance0 * distance1 < 0.0f){
		// Find intersection point of edge and plane
		float32 interp = distance0 / (distance0 - distance1);
		vOut[numOut].v = b2Math.add(vIn[0].v, b2Math.multiply(interp, b2Math.subtract(vIn[1].v, vIn[0].v)));
		if (distance0 > 0.0f){
			vOut[numOut].id = vIn[0].id;
		} else{
			vOut[numOut].id = vIn[1].id;
		}
		++numOut;
	}

	return numOut;
}

static float32 EdgeSeparation(b2PolyShape poly1, int32 edge1, b2PolyShape poly2){
	int32 count1 = poly1.m_vertexCount;
	b2Vec2[] vert1s = poly1.m_vertices;
	int32 count2 = poly2.m_vertexCount;
	b2Vec2[] vert2s = poly2.m_vertices;

	// Get the vertices associated with edge1.
	int32 vertexIndex11 = edge1;
	int32 vertexIndex12 = edge1 + 1 == count1 ? 0 : edge1 + 1;

	// Get the normal of edge1.
	b2Vec2 normalLocal1 = b2Math.b2Cross(b2Math.subtract(vert1s[vertexIndex12], vert1s[vertexIndex11]), 1.0f);
	normalLocal1.Normalize();
	b2Vec2 normal = b2Math.b2Mul(poly1.m_R, normalLocal1);
	b2Vec2 normalLocal2 = b2Math.b2MulT(poly2.m_R, normal);

	// Find support vertex on poly2 for -normal.
	int32 vertexIndex2 = 0;
	float32 minDot = FLT_MAX;
	for (int32 i = 0; i < count2; ++i){
		float32 dot = b2Math.b2Dot(vert2s[i], normalLocal2);
		if (dot < minDot){
			minDot = dot;
			vertexIndex2 = i;
		}
	}

	b2Vec2 v1 = b2Math.add(poly1.m_position, b2Math.b2Mul(poly1.m_R, vert1s[vertexIndex11]));
	b2Vec2 v2 = b2Math.add(poly2.m_position, b2Math.b2Mul(poly2.m_R, vert2s[vertexIndex2]);
	float32 separation = b2Math.b2Dot( b2Math.subtract(v2, v1), normal);
	return separation;
}

// Find the max separation between poly1 and poly2 using face normals
// from poly1.

//Java note: Finally, for the first time, we come across an integer pointer used to
//alter a passed variable, which Java just won't let us do under any circumstances.
//This means that what we need to do is track down where this is called and see if we can find a hack
//around this issue.
//A possibility: declare a mutable integer holder class, require that as the input to this function,
//and wrap calls to this function with creations of that mutable integer and a setting from it.
static float32 FindMaxSeparation(int32* edge, b2PolyShape poly1, b2PolyShape poly2){
	int32 count1 = poly1.m_vertexCount;
	b2Vec2[] vert1s = poly1.m_vertices;

	// Vector pointing from the origin of poly1 to the origin of poly2.
	b2Vec2 d = b2Math.subtract(poly2.m_position, poly1.m_position);
	b2Vec2 dLocal1 = b2Math.b2MulT(poly1.m_R, d);

	// Get support vertex as a hint for our search
	int32 vertexIndex1 = 0;
	float32 maxDot = -FLT_MAX;
	for (int32 i = 0; i < count1; ++i){
		float32 dot = b2Math.b2Dot(vert1s[i], dLocal1);
		if (dot > maxDot){
			maxDot = dot;
			vertexIndex1 = i;
		}
	}

	// Check the separation for the edges straddling the vertex.
	int32 prevFaceIndex = vertexIndex1 - 1 >= 0 ? vertexIndex1 - 1 : count1 - 1;
	float32 sPrev = EdgeSeparation(poly1, prevFaceIndex, poly2);
	if (sPrev > 0.0f){
		return sPrev;
	}

	int32 nextFaceIndex = vertexIndex1;
	float32 sNext = EdgeSeparation(poly1, nextFaceIndex, poly2);
	if (sNext > 0.0f){
		return sNext;
	}

	// Find the best edge and the search direction.
	int32 bestFaceIndex;
	float32 bestSeparation;
	int32 increment;
	if (sPrev > sNext){
		increment = -1;
		bestFaceIndex = prevFaceIndex;
		bestSeparation = sPrev;
	} else{
		increment = 1;
		bestFaceIndex = nextFaceIndex;
		bestSeparation = sNext;
	}

	for ( ; ; ){
		int32 edgeIndex;
		if (increment == -1)
			edgeIndex = bestFaceIndex - 1 >= 0 ? bestFaceIndex - 1 : count1 - 1;
		else
			edgeIndex = bestFaceIndex + 1 < count1 ? bestFaceIndex + 1 : 0;

		float32 separation = EdgeSeparation(poly1, edgeIndex, poly2);
		if (separation > 0.0f){
			return separation;
		}

		if (separation > bestSeparation){
			bestFaceIndex = edgeIndex;
			bestSeparation = separation;
		} else{
			break;
		}
	}

	*edge = bestFaceIndex; //Java note: here is the offending alteration of the passed integer
	return bestSeparation;
}

static void FindIncidentEdge(ClipVertex[] c, b2PolyShape poly1, int32 edge1, b2PolyShape poly2){
	int32 count1 = poly1.m_vertexCount;
	b2Vec2[] vert1s = poly1.m_vertices;
	int32 count2 = poly2.m_vertexCount;
	b2Vec2[] vert2s = poly2.m_vertices;

	// Get the vertices associated with edge1.
	int32 vertex11 = edge1;
	int32 vertex12 = edge1 + 1 == count1 ? 0 : edge1 + 1;

	// Get the normal of edge1.
	b2Vec2 normal1Local1 = b2Math.b2Cross(b2Math.subtract(vert1s[vertex12], vert1s[vertex11]), 1.0f);
	normal1Local1.Normalize();
	b2Vec2 normal1 = b2Math.b2Mul(poly1.m_R, normal1Local1);
	b2Vec2 normal1Local2 = b2Math.b2MulT(poly2.m_R, normal1);

	// Find the incident edge on poly2.
	int32 vertex21 = 0, vertex22 = 0;
	float32 minDot = FLT_MAX;
	for (int32 i = 0; i < count2; ++i){
		int32 i1 = i;
		int32 i2 = i + 1 < count2 ? i + 1 : 0;

		b2Vec2 normal2Local2 = b2Math.b2Cross(b2Math.subtract(vert2s[i2], vert2s[i1]), 1.0f);
		normal2Local2.Normalize();
		float32 dot = b2Math.b2Dot(normal2Local2, normal1Local2);
		if (dot < minDot){
			minDot = dot;
			vertex21 = i1;
			vertex22 = i2;
		}
	}

	// Build the clip vertices for the incident edge.
	c[0].v = b2Math.add(poly2.m_position, b2Math.b2Mul(poly2.m_R, vert2s[vertex21]));
	c[0].id.features.referenceFace = (uint8)edge1; //Java note: look here to see how b2ContactID is used...
	c[0].id.features.incidentEdge = (uint8)vertex21;
	c[0].id.features.incidentVertex = (uint8)vertex21;

	c[1].v = b2Math.add(poly2.m_position, b2Math.b2Mul(poly2.m_R, vert2s[vertex22]));
	c[1].id.features.referenceFace = (uint8)edge1;
	c[1].id.features.incidentEdge = (uint8)vertex21;
	c[1].id.features.incidentVertex = (uint8)vertex22;
}

// Find edge normal of max separation on A - return if separating axis is found
// Find edge normal of max separation on B - return if separation axis is found
// Choose reference edge as min(minA, minB)
// Find incident edge
// Clip

// The normal points from 1 to 2
void b2CollidePoly(b2Manifold manifold, b2PolyShape polyA, b2PolyShape polyB){
	manifold.pointCount = 0;

	int32 edgeA = 0;
	float32 separationA = FindMaxSeparation(&edgeA, polyA, polyB); //Java note: int pointer alert
	if (separationA > 0.0f)
		return;

	int32 edgeB = 0;
	float32 separationB = FindMaxSeparation(&edgeB, polyB, polyA); //Java note: int pointer alert
	if (separationB > 0.0f)
		return;

	b2PolyShape poly1;	// reference poly
	b2PolyShape poly2;	// incident poly
	int32 edge1;		// reference edge
	uint8 flip;
	float32 k_relativeTol = 0.98f;
	float32 k_absoluteTol = 0.001f;

	// TODO_ERIN use "radius" of poly for absolute tolerance.
	if (separationB > k_relativeTol * separationA + k_absoluteTol){
		poly1 = polyB;
		poly2 = polyA;
		edge1 = edgeB;
		flip = 1;
	} else{
		poly1 = polyA;
		poly2 = polyB;
		edge1 = edgeA;
		flip = 0;
	}

	ClipVertex[] incidentEdge = new ClipVertex[2];
	incidentEdge[0] = new ClipVertex();
	incidentEdge[1] = new ClipVertex();
	FindIncidentEdge(incidentEdge, poly1, edge1, poly2);

	int32 count1 = poly1.m_vertexCount;
	b2Vec2[] vert1s = poly1.m_vertices;

	b2Vec2 v11 = new b2Vec2(vert1s[edge1]);
	b2Vec2 v12 = b2Math.add(edge1, 1 < count1 ? vert1s[edge1+1] : vert1s[0]);

	b2Vec2 dv = b2Math.subtract(v12, v11);
	b2Vec2 sideNormal = b2Math.b2Mul(poly1.m_R, b2Math.subtract(v12, v11));
	sideNormal.Normalize();
	b2Vec2 frontNormal = b2Math.b2Cross(sideNormal, 1.0f);
	
	v11 = b2Math.add(poly1.m_position, b2Math.b2Mul(poly1.m_R, v11));
	v12 = b2Math.add(poly1.m_position, b2Math.b2Mul(poly1.m_R, v12));

	float32 frontOffset = b2Math.b2Dot(frontNormal, v11);
	float32 sideOffset1 = -b2Math.b2Dot(sideNormal, v11);
	float32 sideOffset2 = b2Math.b2Dot(sideNormal, v12);

	// Clip incident edge against extruded edge1 side edges.
	ClipVertex[] clipPoints1 = new ClipVertex[2];
	ClipVertex[] clipPoints2 = new ClipVertex[2];
	clipPoints1[0] = new ClipVertex(); clipPoints1[1] = new ClipVertex();
	clipPoints2[0] = new ClipVertex(); clipPoints2[1] = new ClipVertex();
	
	int np;

	// Clip to box side 1
	np = ClipSegmentToLine(clipPoints1, incidentEdge, b2Math.multiply(-1f,sideNormal), sideOffset1);

	if (np < 2)
		return;

	// Clip to negative box side 1
	np = ClipSegmentToLine(clipPoints2, clipPoints1,  sideNormal, sideOffset2);

	if (np < 2)
		return;

	// Now clipPoints2 contains the clipped points.
	manifold.normal = new b2Vec2(flip ? -frontNormal : frontNormal);

	int32 pointCount = 0;
	for (int32 i = 0; i < b2_maxManifoldPoints; ++i){
		float32 separation = b2Math.b2Dot(frontNormal, clipPoints2[i].v) - frontOffset;

		if (separation <= 0.0f){
			b2ContactPoint cp = manifold.points[pointCount];
			cp.separation = separation;
			cp.position = new b2Vec2(clipPoints2[i].v);
			cp.id = new b2ContactID(clipPoints2[i].id);
			cp.id.features.flip = flip;
			++pointCount;
		}
	}

	manifold.pointCount = pointCount;
}

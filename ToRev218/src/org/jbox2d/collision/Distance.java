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

import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.collision.structs.DistanceInput;
import org.jbox2d.collision.structs.DistanceOutput;
import org.jbox2d.collision.structs.SimplexCache;
import org.jbox2d.collision.structs.SimplexVertex;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;

//updated to rev 108->139->218 of Distance.cpp

/** Implements the GJK algorithm for computing distance between shapes. */
public class Distance{
	public static int k_maxIterationCount = 20;

	// djm pooling
	public static final Simplex simplex = new Simplex();
	public static final int[] lastA = new int[4];
	public static final int[] lastB = new int[4];
	public static final Vec2 p = new Vec2();
	public static final Vec2 temp = new Vec2();
	public static final Vec2 normal = new Vec2();
	/**
	 * Compute the closest points between two shapes. Supports any combination of:
	 * CircleShape, PolygonShape, EdgeShape. The simplex cache is input/output.
	 * On the first call set SimplexCache.count to zero.
	 * @param output
	 * @param cache
	 * @param input
	 * @param shapeA
	 * @param shapeB
	 */
	public static final void distance(DistanceOutput output,
					SimplexCache cache, 
					DistanceInput input,
					Shape shapeA,
					Shape shapeB){
		XForm transformA = input.transformA;
		XForm transformB = input.transformB;

		// Initialize the simplex.
		simplex.readCache(cache, shapeA, transformA, shapeB, transformB);

		// Get simplex vertices as an array.
		SimplexVertex[] vertices = simplex.vertices;

		// These store the vertices of the last simplex so that we
		// can check for duplicates and prevent cycling.
		//int lastA[4], lastB[4]; pooled
		int lastCount = 0;

		// Main iteration loop.
		int iter = 0;
		while (iter < k_maxIterationCount){
			// Copy simplex so we can identify duplicates.
			lastCount = simplex.m_count;
			for (int i = 0; i < lastCount; ++i){
				lastA[i] = vertices[i].indexA;
				lastB[i] = vertices[i].indexB;
			}

			switch (simplex.m_count){
			case 1:
				break;

			case 2:
				simplex.solve2();
				break;

			case 3:
				simplex.solve3();
				break;

			default:
				assert(false);
			}

			// If we have 3 points, then the origin is in the corresponding triangle.
			if (simplex.m_count == 3){
				break;
			}

			// Compute closest point.
			p.set(simplex.getClosestPoint());
			float distanceSqr = p.lengthSquared();

			// Ensure the search direction is numerically fit.
			if (distanceSqr < Settings.EPSILON * Settings.EPSILON){
				// The origin is probably contained by a line segment
				// or triangle. Thus the shapes are overlapped.

				// We can't return zero here even though there may be overlap.
				// In case the simplex is a point, segment, or triangle it is difficult
				// to determine if the origin is contained in the CSO or very close to it.
				break;
			}

			// Compute a tentative new simplex vertex using support points.
			SimplexVertex vertex = vertices[simplex.m_count];
			Mat22.mulTransToOut(transformA.R, p, temp);
			vertex.indexA = shapeA.getSupport(temp);
			
			//vertex.wA = Mul(transformA, shapeA.GetVertex(vertex.indexA));
			XForm.mulToOut(transformA, shapeA.getVertex(vertex.indexA), vertex.wA);
			
			//Vec2 wBLocal;
			//vertex.indexB = shapeB.GetSupport(MulT(transformB.R, -p));
			Mat22.mulTransToOut(transformB.R, p.negateLocal(), temp);
			p.negateLocal();
			vertex.indexB = shapeB.getSupport(temp);
			
			//vertex.wB = Mul(transformB, shapeB.GetVertex(vertex.indexB));
			XForm.mulToOut(transformB, shapeB.getVertex(vertex.indexB), vertex.wB);
			vertex.w.set(vertex.wB).subLocal(vertex.wA);

			// Iteration count is equated to the number of support point calls.
			++iter;

			// Check for convergence.
			float lowerBound = Vec2.dot(p, vertex.w);
			float upperBound = distanceSqr;
			final float k_relativeTolSqr = 0.01f * 0.01f;	// 1:100
			if (upperBound - lowerBound <= k_relativeTolSqr * upperBound){
				// Converged!
				break;
			}

			// Check for duplicate support points.
			boolean duplicate = false;
			for (int i = 0; i < lastCount; ++i){
				if (vertex.indexA == lastA[i] && vertex.indexB == lastB[i]){
					duplicate = true;
					break;
				}
			}

			// If we found a duplicate support point we must exit to avoid cycling.
			if (duplicate){
				break;
			}

			// New vertex is ok and needed.
			++simplex.m_count;
		}

		// Prepare output.
		simplex.getWitnessPoints(output.pointA, output.pointB);
		output.distance = MathUtils.distance(output.pointA, output.pointB);
		output.iterations = iter;

		// Cache the simplex.
		simplex.writeCache(cache);

		// Apply radii if requested.
		if (input.useRadii){
			float rA = shapeA.m_radius;
			float rB = shapeB.m_radius;

			if (output.distance > rA + rB && output.distance > Settings.EPSILON){
				// Shapes are still no overlapped.
				// Move the witness points to the outer surface.
				output.distance -= rA + rB;
				normal.set(output.pointB).subLocal(output.pointA);
				normal.normalize();
				//output.pointA += rA * normal;
				//output.pointB -= rB * normal;
				temp.set(normal).mulLocal(rA);
				output.pointA.addLocal(temp);
				temp.set(normal).mulLocal(rB);
				output.pointB.subLocal(temp);
			}
			else{
				// Shapes are overlapped when radii are considered.
				// Move the witness points to the middle.
				//Vec2 p = 0.5f * (output.pointA + output.pointB);
				p.set(output.pointA).addLocal(output.pointB).mulLocal(.5f);
				output.pointA.set(p);
				output.pointB.set(p);
				output.distance = 0.0f;
			}
		}
	}
}
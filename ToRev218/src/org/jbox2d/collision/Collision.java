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

import org.jbox2d.collision.structs.ClipVertex;
import org.jbox2d.collision.structs.ContactID;
import org.jbox2d.collision.structs.PointState;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;

//Updated to rev 139->218 of Collision.h

/** A few variables, classes, and methods that don't fit anywhere else (globals in C++ code). */
public class Collision {
	public static final int NULL_FEATURE = Integer.MAX_VALUE;

	/**
	 * Compute the point states given two manifolds. The states pertain to the transition from manifold1
	 * to manifold2. So state1 is either persist or remove while state2 is either add or persist.
	 * @param state1 this should be an array of length {@link Settings#maxManifoldPoints}
	 * @param state2 this should be an array of length {@link Settings#maxManifoldPoints}
	 * @param manifold1 the previous manifold
	 * @param manifold1 the current manifold
	 */
	public static final void getPointStates(final PointState state1[], final PointState state2[],
	                                        final Manifold manifold1, final Manifold manifold2){
		for (int i = 0; i < Settings.maxManifoldPoints; ++i){
			state1[i] = PointState.nullState;
			state2[i] = PointState.nullState;
		}

		// Detect persists and removes.
		for (int i = 0; i < manifold1.m_pointCount; ++i){
			final ContactID id = manifold1.m_points[i].m_id;

			state1[i] = PointState.removeState;

			for (int j = 0; j < manifold2.m_pointCount; ++j){
				if (manifold2.m_points[j].m_id.key == id.key){
					state1[i] = PointState.persistState;
					break;
				}
			}
		}

		// Detect persists and adds.
		for (int i = 0; i < manifold2.m_pointCount; ++i){
			final ContactID id = manifold2.m_points[i].m_id;

			state2[i] = PointState.addState;

			for (int j = 0; j < manifold1.m_pointCount; ++j){
				if (manifold1.m_points[j].m_id.key == id.key){
					state2[i] = PointState.persistState;
					break;
				}
			}
		}
	}

	/**
	 * Clipping for contact manifolds.  Sutherland-Hodgman clipping.
	 * @param vOut array of length 2
	 * @param vIn array of length 2
	 * @param normal
	 * @param offset
	 * @return
	 */
	public final static int ClipSegmentToLine(final ClipVertex[] vOut, final ClipVertex[] vIn,
	                                          final Vec2 normal, final float offset){
		assert(vOut.length == 2);
		assert(vIn.length == 2);

		// Start with no output points
		int numOut = 0;

		// Calculate the distance of end points to the line
		final float distance0 = Vec2.dot(normal, vIn[0].v) - offset;
		final float distance1 = Vec2.dot(normal, vIn[1].v) - offset;

		// If the points are behind the plane
		if (distance0 <= 0.0f){
			vOut[numOut++] = vIn[0];
		}

		if (distance1 <= 0.0f){
			vOut[numOut++] = vIn[1];
		}

		// If the points are on different sides of the plane
		if (distance0 * distance1 < 0.0f){

			// Find intersection point of edge and plane
			final float interp = distance0 / (distance0 - distance1);
			//vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
			vOut[numOut].v.set( vIn[1].v).subLocal( vIn[0].v).mulLocal( interp);
			vOut[numOut].v.addLocal( vIn[0].v);

			if (distance0 > 0.0f){
				vOut[numOut].id.set(vIn[0].id);
			}
			else{
				vOut[numOut].id.set(vIn[1].id);
			}
			++numOut;
		}

		return numOut;
	}
}

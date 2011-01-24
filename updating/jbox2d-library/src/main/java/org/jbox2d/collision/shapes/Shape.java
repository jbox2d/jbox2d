/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DANIEL MURPHY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
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

package org.jbox2d.collision.shapes;

import org.jbox2d.collision.AABB;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.structs.collision.RayCastInput;
import org.jbox2d.structs.collision.RayCastOutput;

// Updated to rev 100

/**
 * A shape is used for collision detection. You can create a shape however you like.
 * Shapes used for simulation in World are created automatically when a Fixture
 * is created.  Shapes may encapsulate a one or more child shapes.
 */
public abstract class Shape {

	public ShapeType m_type;
	public float m_radius;

	public Shape() {
		m_type = ShapeType.UNKNOWN;
	}
	
	/**
	 * Get the type of this shape. You can use this to down cast to the concrete shape.
	 * @return the shape type.
	 */
	public ShapeType getType() {
		return m_type;
	}

	/**
	 * Test a point for containment in this shape. This only works for convex shapes.
	 * @param xf the shape world transform.
	 * @param p a point in world coordinates.
	 */
	public abstract boolean testPoint( final Transform xf, final Vec2 p);

	/**
	 * Cast a ray against a child shape.
	 * @param argOutput the ray-cast results.
	 * @param argInput the ray-cast input parameters.
	 * @param argTransform the transform to be applied to the shape.
	 * @param argChildIndex the child shape index
	 * @return if hit
	 */
	public abstract boolean raycast( RayCastOutput argOutput, RayCastInput argInput, Transform argTransform);


	/**
	 * Given a transform, compute the associated axis aligned bounding box for a child shape.
	 * @param argAabb returns the axis aligned box.
	 * @param argXf the world transform of the shape.
	 */
	public abstract void computeAABB(final AABB argAabb, final Transform argXf);

	/**
	 * Compute the mass properties of this shape using its dimensions and density.
	 * The inertia tensor is computed about the local origin.
	 * @param massData returns the mass data for this shape.
	 * @param density the density in kilograms per meter squared.
	 */
	public abstract void computeMass(final MassData massData, final float density);
	
	
	/*
	 * Compute the volume and centroid of this shape intersected with a half plane
	 * @param normal the surface normal
	 * @param offset the surface offset along normal
	 * @param xf the shape transform
	 * @param c returns the centroid
	 * @return the total volume less than offset along normal
	 *
	public abstract float computeSubmergedArea(Vec2 normal,
									  float offset,
									  Transform xf, 
									  Vec2 c);*/
	
	
	public abstract Shape clone();
}

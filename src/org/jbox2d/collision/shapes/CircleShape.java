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
import org.jbox2d.collision.MassData;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;
import org.jbox2d.dynamics.Body;


//Updated to rev 56->108->139 of b2Shape.cpp/.h

/**
 * A circle shape.  Create using {@link Body#createShape(ShapeDef)} with a {@link CircleDef},
 * not the constructor here.
 * @see Body#createShape(ShapeDef)
 * @see CircleDef
 */
public class CircleShape extends Shape {

	public float m_radius;
	public Vec2 m_localPosition;

	/**
	 * this is used internally, instead use {@link Body#createShape(ShapeDef)} with a {@link CircleDef}
	 * @see Body#createShape(ShapeDef)
	 * @see CircleDef
	 * @param def
	 */
	public CircleShape(final ShapeDef def) {
		super(def);
		assert(def.type == ShapeType.CIRCLE_SHAPE);
		final CircleDef circleDef = (CircleDef)def;
		m_type = ShapeType.CIRCLE_SHAPE;
		m_localPosition = circleDef.localPosition.clone();
		m_radius = circleDef.radius;
	}

	@Override
	public void updateSweepRadius(final Vec2 center) {
		// Update the sweep radius (maximum radius) as measured from
		// a local center point.
		//Vec2 d = m_localPosition.sub(center);
		final float dx = m_localPosition.x - center.x;
		final float dy = m_localPosition.y - center.y;
		m_sweepRadius = (float)Math.sqrt( dx*dx + dy*dy) + m_radius - Settings.toiSlop;
	}

	/**
	 * checks to see if the point is in this shape.
	 */
	// djm: not a hot method, don't pool
	@Override
	public boolean testPoint(final XForm transform, final Vec2 p) {
		final Vec2 center = new Vec2();
		Mat22.mulToOut(transform.R, m_localPosition, center);
		center.addLocal(transform.position);

		final Vec2 d = center.subLocal(p).negateLocal();
		return Vec2.dot(d, d) <= m_radius * m_radius;
	}

	/*
 // Collision Detection in Interactive 3D Environments by Gino van den Bergen
 // From Section 3.1.2
 // x = s + a * r
 // norm(x) = radius
 bool b2CircleShape::TestSegment(const b2XForm& transform,
 								float32* lambda,
 								b2Vec2* normal,
 								const b2Segment& segment,
 								float32 maxLambda) const
 {
 	b2Vec2 position = transform.position + b2Mul(transform.R, m_localPosition);
 	b2Vec2 s = segment.p1 - position;
 	float32 b = b2Dot(s, s) - m_radius * m_radius;

 	// Does the segment start inside the circle?
 	if (b < 0.0f)
 	{
 		return false;
 	}

 	// Solve quadratic equation.
 	b2Vec2 r = segment.p2 - segment.p1;
 	float32 c =  b2Dot(s, r);
 	float32 rr = b2Dot(r, r);
 	float32 sigma = c * c - rr * b;

 	// Check for negative discriminant and short segment.
 	if (sigma < 0.0f || rr < FLT_EPSILON)
 	{
 		return false;
 	}

 	// Find the point of intersection of the line with the circle.
 	float32 a = -(c + sqrtf(sigma));

 	// Is the intersection point on the segment?
 	if (0.0f <= a && a <= maxLambda * rr)
 	{
 		a /= rr;
	 *lambda = a;
	 *normal = s + a * r;
 		normal->Normalize();
 		return true;
 	}

 	return false;
 }*/

	/**
	 * @see Shape#computeAABB(AABB, XForm)
	 */
	// djm: keeping the Vec2 creation here, it's not a hot method
	@Override
	public void computeAABB(final AABB aabb, final XForm transform) {
		final Vec2 p = Mat22.mul(transform.R, m_localPosition);
		p.addLocal(transform.position);

		aabb.lowerBound.x = p.x - m_radius;
		aabb.lowerBound.y = p.y - m_radius;
		aabb.upperBound.x = p.x + m_radius;
		aabb.upperBound.y = p.y + m_radius;
	}

	/**
	 * @see Shape#computeSweptAABB(AABB, XForm, XForm)
	 */
	@Override
	public void computeSweptAABB(final AABB aabb, final XForm transform1, final XForm transform2) {
		// INLINED
		//Vec2 p1 = transform1.position.add(Mat22.mul(transform1.R, m_localPosition));
		//Vec2 p2 = transform2.position.add(Mat22.mul(transform2.R, m_localPosition));
		//Vec2 lower = Vec2.min(p1, p2);
		//Vec2 upper = Vec2.max(p1, p2);
		//aabb.lowerBound.set(lower.x - m_radius, lower.y - m_radius);
		//aabb.upperBound.set(upper.x + m_radius, upper.y + m_radius);
		final float p1x = transform1.position.x + transform1.R.col1.x * m_localPosition.x + transform1.R.col2.x * m_localPosition.y;
		final float p1y = transform1.position.y + transform1.R.col1.y * m_localPosition.x + transform1.R.col2.y * m_localPosition.y;
		final float p2x = transform2.position.x + transform2.R.col1.x * m_localPosition.x + transform2.R.col2.x * m_localPosition.y;
		final float p2y = transform2.position.y + transform2.R.col1.y * m_localPosition.x + transform2.R.col2.y * m_localPosition.y;
		final float lowerx = p1x < p2x ? p1x : p2x;
		final float lowery = p1y < p2y ? p1y : p2y;
		final float upperx = p1x > p2x ? p1x : p2x;
		final float uppery = p1y > p2y ? p1y : p2y;
		aabb.lowerBound.x = lowerx - m_radius;
		aabb.lowerBound.y = lowery - m_radius;
		aabb.upperBound.x = upperx + m_radius;
		aabb.upperBound.y = uppery + m_radius;
		//System.out.println("Circle swept AABB: " + aabb.lowerBound + " " + aabb.upperBound);
		//System.out.println("Transforms: "+transform1.position+ " " + transform2.position+"\n");

	}

	@Override
	public void computeMass(final MassData massData) {
		massData.mass = m_density * (float)Math.PI * m_radius * m_radius;
		massData.center = m_localPosition.clone();

		// inertia about the local origin
		massData.I = massData.mass * (0.5f * m_radius * m_radius + Vec2.dot(m_localPosition, m_localPosition));
	}


	public float getRadius() {
		return m_radius;
	}

	/**
	 * Returns a copy of the local position
	 * @return
	 */
	public Vec2 getLocalPosition() {
		return m_localPosition.clone();
	}

	/**
	 * Returns the member variable of the local position.
	 * Don't change this.
	 * @return
	 */
	public Vec2 getMemberLocalPosition(){
		return m_localPosition;
	}
}

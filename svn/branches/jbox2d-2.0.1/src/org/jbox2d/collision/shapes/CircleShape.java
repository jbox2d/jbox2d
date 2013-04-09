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
import org.jbox2d.collision.Segment;
import org.jbox2d.collision.SegmentCollide;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.RaycastResult;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;
import org.jbox2d.dynamics.Body;
import org.jbox2d.pooling.TLVec2;

//Updated to rev 56->108->139 of b2Shape.cpp/.h

/**
 * A circle shape. Create using {@link Body#createShape(ShapeDef)} with a
 * {@link CircleDef}, not the constructor here.
 * 
 * @see Body#createShape(ShapeDef)
 * @see CircleDef
 */
public class CircleShape extends Shape {

	public float m_radius;
	public final Vec2 m_localPosition;

	/**
	 * this is used internally, instead use {@link Body#createShape(ShapeDef)}
	 * with a {@link CircleDef}
	 * 
	 * @see Body#createShape(ShapeDef)
	 * @see CircleDef
	 * @param def
	 */
	public CircleShape(final ShapeDef def) {
		super(def);
		assert (def.type == ShapeType.CIRCLE_SHAPE);
		final CircleDef circleDef = (CircleDef) def;
		m_type = ShapeType.CIRCLE_SHAPE;
		m_localPosition = circleDef.localPosition.clone();
		m_radius = circleDef.radius;
	}

	/**
	 * @see Shape#updateSweepRadius(Vec2)
	 */
	@Override
	public void updateSweepRadius(final Vec2 center) {
		// Update the sweep radius (maximum radius) as measured from
		// a local center point.
		// Vec2 d = m_localPosition.sub(center);
		final float dx = m_localPosition.x - center.x;
		final float dy = m_localPosition.y - center.y;
		m_sweepRadius = MathUtils.sqrt(dx * dx + dy * dy) + m_radius
				- Settings.toiSlop;
	}

	// djm pooling
	private static final TLVec2 tlCenter = new TLVec2();	
	/**
	 * checks to see if the point is in this shape.
	 * 
	 * @see Shape#testPoint(XForm, Vec2)
	 */
	@Override
	public boolean testPoint(final XForm transform, final Vec2 p) {
		final Vec2 center = tlCenter.get();
		Mat22.mulToOut(transform.R, m_localPosition, center);
		center.addLocal(transform.position);

		final Vec2 d = center.subLocal(p).negateLocal();
		boolean ret = Vec2.dot(d, d) <= m_radius * m_radius;
		return ret;
	}

	
	// djm pooling
	private static final TLVec2 tlS = new TLVec2();
	private static final TLVec2 tlPosition = new TLVec2();
	private static final TLVec2 tlR = new TLVec2();
	// Collision Detection in Interactive 3D Environments by Gino van den Bergen
	// From Section 3.1.2
	// x = s + a * r
	// norm(x) = radius
	/**
	 * @see Shape#testSegment(XForm, RaycastResult, Segment, float)
	 */
	@Override
	public SegmentCollide testSegment(final XForm xf, final RaycastResult out,
			final Segment segment, final float maxLambda) {
		Vec2 position = tlPosition.get();
		Vec2 s = tlS.get();
		
		Mat22.mulToOut(xf.R, m_localPosition, position);
		position.addLocal(xf.position);
		s.set(segment.p1);
		s.subLocal(position);
		final float b = Vec2.dot(s, s) - m_radius * m_radius;

		// Does the segment start inside the circle?
		if (b < 0.0f) {
			return SegmentCollide.STARTS_INSIDE_COLLIDE;
		}

		Vec2 r = tlR.get();
		// Solve quadratic equation.
		r.set(segment.p2).subLocal(segment.p1);
		final float c = Vec2.dot(s, r);
		final float rr = Vec2.dot(r, r);
		final float sigma = c * c - rr * b;

		// Check for negative discriminant and short segment.
		if (sigma < 0.0f || rr < Settings.EPSILON) {
			return SegmentCollide.MISS_COLLIDE;
		}

		// Find the point of intersection of the line with the circle.
		float a = -(c + MathUtils.sqrt(sigma));

//		 System.out.println(a + "; " + maxLambda + "; " + rr + "  ; " + (a <= maxLambda * rr));

		// Is the intersection point on the segment?
		if (0.0f <= a && a <= maxLambda * rr) {
			// System.out.println("Got here");
			a /= rr;
			out.lambda = a;
			out.normal.set(r).mulLocal(a).addLocal(s);
			out.normal.normalize();
			
			return SegmentCollide.HIT_COLLIDE;
		}

		return SegmentCollide.MISS_COLLIDE; // thanks FrancescoITA
	}

	// djm pooling
	private static final TLVec2 tlP = new TLVec2();
	/**
	 * @see Shape#computeAABB(AABB, XForm)
	 */
	@Override
	public void computeAABB(final AABB aabb, final XForm transform) {
		
		final Vec2 p = tlP.get();
		Mat22.mulToOut(transform.R, m_localPosition, p);
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
	public void computeSweptAABB(final AABB aabb, final XForm transform1,
			final XForm transform2) {
		// INLINED
//		 Vec2 p1 = transform1.position.add(Mat22.mul(transform1.R,
//		 m_localPosition));
//		 Vec2 p2 = transform2.position.add(Mat22.mul(transform2.R,
//		 m_localPosition));
//		 Vec2 lower = Vec2.min(p1, p2);
//		 Vec2 upper = Vec2.max(p1, p2);
//		 aabb.lowerBound.set(lower.x - m_radius, lower.y - m_radius);
//		 aabb.upperBound.set(upper.x + m_radius, upper.y + m_radius);
		final float p1x = transform1.position.x + transform1.R.col1.x
				* m_localPosition.x + transform1.R.col2.x * m_localPosition.y;
		final float p1y = transform1.position.y + transform1.R.col1.y
				* m_localPosition.x + transform1.R.col2.y * m_localPosition.y;
		final float p2x = transform2.position.x + transform2.R.col1.x
				* m_localPosition.x + transform2.R.col2.x * m_localPosition.y;
		final float p2y = transform2.position.y + transform2.R.col1.y
				* m_localPosition.x + transform2.R.col2.y * m_localPosition.y;
		final float lowerx = p1x < p2x ? p1x : p2x;
		final float lowery = p1y < p2y ? p1y : p2y;
		final float upperx = p1x > p2x ? p1x : p2x;
		final float uppery = p1y > p2y ? p1y : p2y;
		aabb.lowerBound.x = lowerx - m_radius;
		aabb.lowerBound.y = lowery - m_radius;
		aabb.upperBound.x = upperx + m_radius;
		aabb.upperBound.y = uppery + m_radius;
//		 System.out.println("Circle swept AABB: " + aabb.lowerBound + " " +
//		 aabb.upperBound);
		// System.out.println("Transforms: "+transform1.position+ " " +
		// transform2.position+"\n");

	}

	/**
	 * @see Shape#computeMass(MassData)
	 */
	@Override
	public void computeMass(final MassData massData) {
		massData.mass = m_density * MathUtils.PI * m_radius * m_radius;
		massData.center.set(m_localPosition);

		// inertia about the local origin
		massData.I = massData.mass
				* (0.5f * m_radius * m_radius + Vec2.dot(m_localPosition,
						m_localPosition));
	}

	public float getRadius() {
		return m_radius;
	}

	/**
	 * Returns a copy of the local position
	 * 
	 * @return
	 */
	public Vec2 getLocalPosition() {
		return m_localPosition.clone();
	}

	/**
	 * Returns the member variable of the local position. Don't change this.
	 * 
	 * @return
	 */
	public Vec2 getMemberLocalPosition() {
		return m_localPosition;
	}
	
	// djm pooling from above
	/**
	 * @see Shape#computeSubmergedArea(Vec2, float, XForm, Vec2)
	 */
	public float computeSubmergedArea(final Vec2 normal, float offset,
			XForm xf, Vec2 c) {
		// pooling
		final Vec2 p = tlP.get();
		
		XForm.mulToOut(xf, m_localPosition, p);
		float l = -(Vec2.dot(normal, p) - offset);
		
		if (l < -m_radius + Settings.EPSILON) {
			// Completely dry
			return 0;
		}
		if (l > m_radius) {
			// Completely wet
			c.set(p);
			return Settings.pi * m_radius * m_radius;
		}

		// Magic
		float r2 = m_radius * m_radius;
		float l2 = l * l;
		float area = (float) (r2
				* (Math.asin(l / m_radius) + Settings.pi / 2.0f) + l
				* MathUtils.sqrt(r2 - l2));
		float com = (float) (-2.0f / 3.0f * MathUtils.pow(r2 - l2, 1.5f) / area);

		c.x = p.x + normal.x * com;
		c.y = p.y + normal.y * com;

		return area;
	}
}

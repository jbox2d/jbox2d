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
import org.jbox2d.collision.structs.SegmentCollide;
import org.jbox2d.collision.structs.ShapeType;
import org.jbox2d.collision.structs.TestSegmentResult;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;
import org.jbox2d.dynamics.Body;

//Updated to rev 56->108->139->[221 of groundzero] of Shape.cpp/.h

/**
 * A circle shape.
 */
public class CircleShape extends Shape {

	private final Vec2 m_localPosition = new Vec2();
	private float m_radius;

	/**
	 * This is used internally, instead use {@link Body#createShape(ShapeDef)} with a {@link CircleDef}
	 * @see Body#createShape(ShapeDef)
	 * @see CircleDef
	 * @param def
	 */
	protected CircleShape(ShapeDef def){
		super(def);
		assert(def.type == ShapeType.CIRCLE_SHAPE);
		m_type = ShapeType.CIRCLE_SHAPE;
		
		CircleDef circleDef = (CircleDef) def;
		
		m_localPosition.set(circleDef.localPosition);
		m_radius = circleDef.radius;
	}
	
	/**
	 * Gets the local position. not a copy.
	 * @return
	 */
	public Vec2 getLocalPosition(){
		return m_localPosition;
	}
	
	/**
	 * returns the radius of this shape.
	 * @return
	 */
	public float getRadius(){
		return m_radius;
	}

	// djm pooling
	public final static Vec2 center = new Vec2();
	/**
	 * @see Shape#testPoint(XForm, Vec2)
	 */
	@Override
	public final boolean testPoint(final XForm transform, final Vec2 p) {
		Mat22.mulToOut(transform.R, m_localPosition, center);
		center.addLocal(transform.position);

		final Vec2 d = center.subLocal(p).negateLocal();
		return Vec2.dot(d, d) <= m_radius * m_radius;
	}


	// Collision Detection in Interactive 3D Environments by Gino van den Bergen
	// From Section 3.1.2
	// x = s + a * r
	// norm(x) = radius
	// djm pooled
	private static final Vec2 position = new Vec2();
	private static final Vec2 s = new Vec2();
	private static final Vec2 r = new Vec2();
	/**
	 * @see Shape#testSegment(XForm, TestSegmentResult, Segment, float)
	 */
	@Override
	public final SegmentCollide testSegment(final XForm transform, final TestSegmentResult out, final Segment segment, final float maxLambda){
		Mat22.mulToOut( transform.R, m_localPosition, position);
		position.addLocal( transform.position);
		s.set( segment.p1).subLocal(position);
		final float b = Vec2.dot( s, s) - m_radius * m_radius;

		// Does the segment start inside the circle?
		if (b < 0.0f){
			out.lambda = 0;
			return SegmentCollide.STARTS_INSIDE_COLLIDE;
		}

		// Solve quadratic equation.
		r.set(segment.p2).subLocal(segment.p1);
		final float c =  Vec2.dot(s, r);
		final float rr = Vec2.dot(r, r);
		final float sigma = c * c - rr * b;

		// Check for negative discriminant and short segment.
		if (sigma < 0.0f || rr < Settings.EPSILON){
			return SegmentCollide.MISS_COLLIDE;
		}

		// Find the point of intersection of the line with the circle.
		float a = -(c + (float)Math.sqrt(sigma));

		// Is the intersection point on the segment?
		if (0.0f <= a && a <= maxLambda * rr){
			a /= rr;
			out.lambda = a;
			out.normal.set(r).mulLocal( a);
			out.normal.addLocal(s);
			out.normal.normalize();
			return SegmentCollide.HIT_COLLIDE;
		}

		return SegmentCollide.MISS_COLLIDE;
	}


	// djm pooling
	private static final Vec2 p = new Vec2();
	/**
	 * @see Shape#computeAABB(AABB, XForm)
	 */
	@Override
	public final void computeAABB(final AABB aabb, final XForm transform) {
		Mat22.mulToOut(transform.R, m_localPosition, p);
		p.addLocal(transform.position);

		aabb.lowerBound.x = p.x - m_radius;
		aabb.lowerBound.y = p.y - m_radius;
		aabb.upperBound.x = p.x + m_radius;
		aabb.upperBound.y = p.y + m_radius;
	}
	
	// djm pooling
	private static final Vec2 p1 = new Vec2();
	private static final Vec2 p2 = new Vec2();
	private static final Vec2 lower = new Vec2();
	private static final Vec2 upper = new Vec2();
	/**
	 * @see Shape#computeSweptAABB(AABB, XForm, XForm)
	 */
	public final void computeSweptAABB(AABB aabb, XForm transform1, XForm transform2){
		Mat22.mulToOut( transform1.R, m_localPosition, p1);
		p1.addLocal(transform1.position);
		
		Mat22.mulToOut( transform2.R, m_localPosition, p2);
		p1.addLocal(transform2.position);
		
		// TODO djm, create method to send max to one var and min to another
		Vec2.minToOut( p1, p2, lower);
		Vec2.maxToOut( p1, p2, upper);
		
		aabb.lowerBound.set(lower.x - m_radius, lower.y - m_radius);
		aabb.upperBound.set(upper.x + m_radius, upper.y + m_radius);
	}

	/**
	 * @see Shape#computeMass(MassData, float)
	 */
	@Override
	public final void computeMass(final MassData massData, final float density) {
		massData.mass = density * MathUtils.PI * m_radius * m_radius;
		massData.center.set(m_localPosition);

		// inertia about the local origin
		massData.I = massData.mass * (0.5f * m_radius * m_radius + Vec2.dot(m_localPosition, m_localPosition));
	}

	// djm pooled from above
	/**
	 * @see Shape#computeSubmergedArea(Vec2, float, Vec2, Vec2)
	 */
	@Override
	public final float computeSubmergedArea( final Vec2 normal, final float offset, final XForm xf, final Vec2 c) {
		XForm.mulToOut(xf,m_localPosition, p);
		final float l = -( Vec2.dot(normal,p) - offset);
		if( l < -m_radius + Settings.EPSILON){
			//Completely dry
			return 0;
		}

		if(l > m_radius){
			//Completely wet
			c.set(p);
			return (float)Math.PI*m_radius*m_radius;
		}

		//Magic
		final float r2 = m_radius*m_radius;
		final float l2 = l*l;

		final float area = (float) (r2 * (Math.asin(l/m_radius) + Math.PI/2)+ l * Math.sqrt(r2 - l2));
		final float com = (float) (-2.0/3.0* Math.pow(r2-l2,1.5f)/area);

		c.x = p.x + normal.x * com;
		c.y = p.y + normal.y * com;

		return area;
	}

	/**
	 * @see Shape#computeSweepRadius(Vec2)
	 */
	@Override
	protected final void updateSweepRadius( final Vec2 center){
		float dx = m_localPosition.x - center.x;
		float dy = m_localPosition.y - center.y;
		
		m_sweepRadius = (float)Math.sqrt(dx*dx + dy*dy) + m_radius - Settings.toiSlop;
	}
}

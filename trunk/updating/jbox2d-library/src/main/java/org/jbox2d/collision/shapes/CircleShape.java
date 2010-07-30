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

import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.pooling.TLVec2;
import org.jbox2d.structs.collision.RayCastInput;
import org.jbox2d.structs.collision.RayCastOutput;


//Updated to rev 100

/**
 * A circle shape.
 */
public class CircleShape extends Shape {

	public final Vec2 m_p; 

	/**
	 * this is used internally, instead use {@link Body#createShape(ShapeDef)} with a {@link CircleDef}
	 * @see Body#createShape(ShapeDef)
	 * @see CircleDef
	 * @param def
	 */
	public CircleShape(){
		m_type = ShapeType.CIRCLE;
		m_p = new Vec2();
		m_radius = 0;
	}
	
	public final Shape clone(){
		CircleShape shape = new CircleShape();
		shape.m_p.set(m_p);
		shape.m_radius = m_radius;
		return shape;
	}
	
	/**
	 * Get the supporting vertex index in the given direction.
	 * @param d
	 * @return
	 */
	public final int getSupport(final Vec2 d){
		return 0;
	}

	/**
	 * Get the supporting vertex in the given direction.
	 * @param d
	 * @return
	 */
	public final Vec2 getSupportVertex( final Vec2 d){
		return m_p;
	}

	/**
	 * Get the vertex count.
	 * @return
	 */
	public final int getVertexCount()  {
		return 1;
	}

	/**
	 * Get a vertex by index. Used by Distance.
	 * @param index
	 * @return
	 */
	public final Vec2 getVertex(final int index){
		assert(index == 0);
		return m_p;
	}

	// djm pooling
	private final static TLVec2 tlcenter = new TLVec2();
	/**
	 * @see Shape#testPoint(Transform, Vec2)
	 */
	@Override
	public final boolean testPoint(final Transform transform, final Vec2 p) {
		final Vec2 center = tlcenter.get();
		Mat22.mulToOut(transform.R, m_p, center);
		center.addLocal(transform.position);

		final Vec2 d = center.subLocal(p).negateLocal();
		return Vec2.dot(d, d) <= m_radius * m_radius;
	}


	// Collision Detection in Interactive 3D Environments by Gino van den Bergen
	// From Section 3.1.2
	// x = s + a * r
	// norm(x) = radius
	// djm pooled
	private static final TLVec2 tlposition = new TLVec2();
	private static final TLVec2 tls = new TLVec2();
	private static final TLVec2 tlr = new TLVec2();
	/**
	 * @see Shape#raycast(org.jbox2d.structs.collision.RayCastOutput, org.jbox2d.structs.collision.RayCastInput, org.jbox2d.common.Transform, int)
	 */
	@Override
	public final boolean raycast( RayCastOutput argOutput, RayCastInput argInput, Transform argTransform){
		
		final Vec2 position = tlposition.get();;
		final Vec2 s = tls.get();
		final Vec2 r = tlr.get();
		
		Mat22.mulToOut( argTransform.R, m_p, position);
		position.addLocal( argTransform.position);
		s.set( argInput.p1).subLocal(position);
		final float b = Vec2.dot( s, s) - m_radius * m_radius;

		// Solve quadratic equation.
		r.set(argInput.p2).subLocal(argInput.p1);
		final float c =  Vec2.dot(s, r);
		final float rr = Vec2.dot(r, r);
		final float sigma = c * c - rr * b;

		// Check for negative discriminant and short segment.
		if (sigma < 0.0f || rr < Settings.EPSILON){
			return false;
		}

		// Find the point of intersection of the line with the circle.
		float a = -(c + MathUtils.sqrt(sigma));

		// Is the intersection point on the segment?
		if (0.0f <= a && a <= argInput.maxFraction * rr){
			a /= rr;
			argOutput.fraction = a;
			argOutput.normal.set(r).mulLocal( a);
			argOutput.normal.addLocal(s);
			argOutput.normal.normalize();
			return true;
		}
		
		return false;
	}


	// djm pooling
	private static final TLVec2 tlp = new TLVec2();
	/**
	 * @see org.jbox2d.collision.shapes.Shape#computeAABB(org.jbox2d.collision.AABB, org.jbox2d.common.Transform, int)
	 */
	@Override
	public final void computeAABB(final AABB argAabb, final Transform argTransform) {
		final Vec2 p = tlp.get();
		Mat22.mulToOut(argTransform.R, m_p, p);
		p.addLocal(argTransform.position);

		argAabb.lowerBound.x = p.x - m_radius;
		argAabb.lowerBound.y = p.y - m_radius;
		argAabb.upperBound.x = p.x + m_radius;
		argAabb.upperBound.y = p.y + m_radius;
	}

	/**
	 * @see Shape#computeMass(MassData, float)
	 */
	@Override
	public final void computeMass(final MassData massData, final float density) {
		massData.mass = density * Settings.PI * m_radius * m_radius;
		massData.center.set(m_p);

		// inertia about the local origin
		massData.I = massData.mass * (0.5f * m_radius * m_radius + Vec2.dot(m_p, m_p));
	}

	// djm pooled from above
	/*
	 * @see Shape#computeSubmergedArea(Vec2, float, Vec2, Vec2)
	 *
	@Override
	public final float computeSubmergedArea( final Vec2 normal, final float offset, final Transform xf, final Vec2 c) {
		final Vec2 p = tlp.get();
		Transform.mulToOut(xf,m_p, p);
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
		//Erin TODO: write Sqrt to handle fixed point case.
		final float area = (float) (r2 * (Math.asin(l/m_radius) + Math.PI/2)+ l * Math.sqrt(r2 - l2));
		final float com = (float) (-2.0/3.0* Math.pow(r2-l2,1.5f)/area);

		c.x = p.x + normal.x * com;
		c.y = p.y + normal.y * com;

		return area;
	}*/
}

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
import org.jbox2d.pooling.TLVec2;
import org.jbox2d.structs.collision.TestSegmentResult;

// Box2d rev 218 Collision.h/.cpp

/**
 * A line segment.
 */
public class Segment {
	
	/**
	 * The starting point
	 */
	public final Vec2 p1 = new Vec2();
	/**
	 * The ending point
	 */
	public final Vec2 p2 = new Vec2();
	
	/*
	// Collision Detection in Interactive 3D Environments by Gino van den Bergen
	// From Section 3.4.1
	// x = mu1 * p1 + mu2 * p2
	// mu1 + mu2 = 1 && mu1 >= 0 && mu2 >= 0
	// mu1 = 1 - mu2;
	// x = (1 - mu2) * p1 + mu2 * p2
	//   = p1 + mu2 * (p2 - p1)
	// x = s + a * r (s := start, r := end - start)
	// s + a * r = p1 + mu2 * d (d := p2 - p1)
	// -a * r + mu2 * d = b (b := s - p1)
	// [-r d] * [a; mu2] = b
	// Cramer's rule:
	// denom = det[-r d]
	// a = det[b d] / denom
	// mu2 = det[-r b] / denom*/
	
	// djm: pooled
	private static final TLVec2 tlr = new TLVec2();
	private static final TLVec2 tld = new TLVec2();
	private static final TLVec2 tln = new TLVec2();
	private static final TLVec2 tlb = new TLVec2();
	/**
	 * Ray cast against this segment with another segment.
	 * @param out
	 * @param segment
	 * @param maxLambda
	 * @return
	 */
	public boolean testSegment(TestSegmentResult out, Segment segment, float maxLambda){
		
		final Vec2 r = tlr.get();
		final Vec2 d = tld.get();
		final Vec2 n = tln.get();
		final Vec2 b = tlb.get();
		
		Vec2 s = segment.p1;
		r.set(segment.p2).subLocal(s);
		d.set(p2).subLocal(p1);
		Vec2.crossToOut( d, 1.0f, n);

		float k_slop = 100.0f * Settings.EPSILON;
		float denom = -Vec2.dot(r, n);

		// Cull back facing collision and ignore parallel segments.
		if (denom > k_slop){
			// Does the segment intersect the infinite line associated with this segment?
			b.set(s).subLocal(p1);
			float a = Vec2.dot(b, n);

			if (0.0f <= a && a <= maxLambda * denom){
				float mu2 = -r.x * b.y + r.y * b.x;

				// Does the segment intersect this segment?
				if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0f + k_slop)){
					a /= denom;
					n.normalize();
					out.lambda = a;
					out.normal.set(n);
					return true;
				}
			}
		}
		
		return false;
	}
}

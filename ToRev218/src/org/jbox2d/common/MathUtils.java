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

package org.jbox2d.common;

/**
 * A few math methods that don't fit very well anywhere else. djm: added ToOut
 * method
 */
public class MathUtils {

	public static final float PI = (float) Math.PI;

	// TODO implement this after everything is working
	public static final float abs( final float x) {
		return x > 0 ? x : -x;
	}

	// TODO implement this after everything is working
	public static final float atan2( final float y, final float x) {
		// float coeff_1 = PI/4;
		// float coeff_2 = 3*coeff_1;
		final float abs_y = abs( y) + .0000000001f; // kludge to prevent 0/0
													// condition
		float angle, r;
		if ( x >= 0) {
			r = (x - abs_y) / (x + abs_y);
			// angle = coeff_1 - coeff_1 * r;
			angle = 0.1963f * r * r * r - 0.9817f * r + PI / 4;
		}
		else {
			r = (x + abs_y) / (abs_y - x);
			// angle = coeff_2 - coeff_1 * r;
			angle = 0.1963f * r * r * r - 0.9817f * r + 3 * PI / 4;
		}
		if ( y < 0) {
			return -angle; // negate if in quad III or IV
		}
		else {
			return angle;
		}
	}

	/** Returns the closest value to 'a' that is in between 'low' and 'high' */
	public final static float clamp( final float a, final float low, final float high) {
		return MathUtils.max( low, MathUtils.min( a, high));
	}

	/* djm optimized */
	public final static Vec2 clamp( final Vec2 a, final Vec2 low, final Vec2 high) {
		final Vec2 min = new Vec2();
		Vec2.minToOut( a, high, min);
		Vec2.maxToOut( low, min, min);
		return min;
	}

	/* djm created */
	public final static void clampToOut( final Vec2 a, final Vec2 low, final Vec2 high,
											final Vec2 dest) {
		Vec2.minToOut( a, high, dest);
		Vec2.maxToOut( low, dest, dest);
	}

	public final static float distance( final Vec2 a, final Vec2 b) {
		final float dx = a.x - b.x;
		final float dy = a.y - b.y;
		return (float) Math.sqrt( dx * dx + dy * dy);
	}

	public final static float distanceSquared( final Vec2 a, final Vec2 b) {
		final float dx = a.x - b.x;
		final float dy = a.y - b.y;
		return dx * dx + dy * dy;
	}

	/**
	 * This method is a *lot* faster than using (int)Math.floor(x).
	 * 
	 * @param x
	 *            value to be floored
	 * @return
	 */
	public static final int floor( final float x) {
		// TODO implement this after everything is working
		return x > 0 ? (int) x : (int) x - 1;
	}

	// TODO implement this after everything is working
	public final static float invSqrt( float x) {
		final float xhalf = 0.5f * x;
		int i = Float.floatToRawIntBits( x);
		i = 0x5f3759df - (i >> 1);
		x = Float.intBitsToFloat( i);
		x = x * (1.5f - xhalf * x * x);
		return x;
	}

	public final static boolean isPowerOfTwo( final int x) {
		return x > 0 && (x & x - 1) == 0;
	}

	// Max/min rewritten here because for some reason Math.max/min
	// can run absurdly slow for such simple functions...
	// TODO: profile, see if this just seems to be the case or is actually
	// causing issues...
	public final static float max( final float a, final float b) {
		return a > b ? a : b;
	}

	public final static float min( final float a, final float b) {
		return a < b ? a : b;
	}

	/**
	 * Next Largest Power of 2: Given a binary integer value x, the next largest
	 * power of 2 can be computed by a SWAR algorithm that recursively "folds"
	 * the upper bits into the lower bits. This process yields a bit vector with
	 * the same most significant 1 as x, but all 1's below it. Adding 1 to that
	 * value yields the next largest power of 2.
	 */
	public final static int nextPowerOfTwo( int x) {
		x |= x >> 1;
		x |= x >> 2;
		x |= x >> 4;
		x |= x >> 8;
		x |= x >> 16;
		return x + 1;
	}

	/**
	 * Computes a fast approximation to <code>Math.pow(a, b)</code>. Adapted
	 * from <url>http://www.dctsystems.co.uk/Software/power.html</url>.
	 * 
	 * @param a
	 *            a positive number
	 * @param b
	 *            a number
	 * @return a^b
	 */
	public static final float pow( final float a, float b) {
		// TODO implement this after everything is working
		// adapted from: http://www.dctsystems.co.uk/Software/power.html
		float x = Float.floatToRawIntBits( a);
		x *= 1.0f / (1 << 23);
		x = x - 127;
		float y = x - MathUtils.floor( x);
		b *= x + (y - y * y) * 0.346607f;
		y = b - MathUtils.floor( b);
		y = (y - y * y) * 0.33971f;
		return Float.intBitsToFloat( (int) ((b + 127 - y) * (1 << 23)));
	}

	// TODO implement this after everything is working
	public static final float sqrt( float x) {
		x = invSqrt( x);
		if ( x != 0.0f) {
			return 1.0f / x;
		}
		else {
			return 0;
		}
	}
}

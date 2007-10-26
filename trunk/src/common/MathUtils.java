/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.gphysics.com
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
package common;

public class MathUtils {
    public static final float clamp(float a, float low, float high) {
        return Math.max(low, Math.min(a, high));
    }

    public static final Vec2 clamp(Vec2 a, Vec2 low, Vec2 high) {
        return Vec2.max(low, Vec2.min(a, high));
    }

    // "Next Largest Power of 2
    // Given a binary integer value x, the next largest power of 2 can be
    // computed by a SWAR algorithm
    // that recursively "folds" the upper bits into the lower bits. This process
    // yields a bit vector with
    // the same most significant 1 as x, but all 1's below it. Adding 1 to that
    // value yields the next
    // largest power of 2. For a 32-bit value:"
    public static final int nextPowerOfTwo(int x) {
        x |= (x >> 1);
        x |= (x >> 2);
        x |= (x >> 4);
        x |= (x >> 8);
        x |= (x >> 16);
        return x + 1;
    }

    public static final boolean isPowerOfTwo(int x) {
        return x > 0 && (x & (x - 1)) == 0;
    }

}

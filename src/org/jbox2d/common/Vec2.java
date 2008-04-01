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
 * A 2-dimensional vector class.  Used heavily in JBox2d.
 *
 */
public class Vec2 {
	/** Should we count Vec2 creations? */
	final static public boolean watchCreations = true;
	/** 
	 * Running count of Vec2 creations.  Must be zeroed out
	 * manually (perhaps at start of time step).  Incremented
	 * in Vec2 constructor if watchCreations flag is true.
	 * <BR><BR>
	 * Mainly used for optimization purposes, since temporary
	 * Vec2 creation is often a bottleneck.
	 */
	static public int creationCount = 0;
	
    public float x, y;

    public Vec2() {
        this(0, 0);
    }

    public Vec2(float x, float y) {
    	if (watchCreations) ++creationCount;
        this.x = x;
        this.y = y;
        // testbed.PTest.debugCount++;
    }

    /** Zero out this vector. */
    public void setZero() {
        x = 0.0f;
        y = 0.0f;
    }

    /** Set the vector component-wise. */
    public void set(float x, float y) {
        this.x = x;
        this.y = y;
    }

    /** Set this vector to another vector. */
    public void set(Vec2 v) {
        this.x = v.x;
        this.y = v.y;
    }

    /** Return the sum of this vector and another; does not alter either one. */
    public Vec2 add(Vec2 v) {
        return new Vec2(x + v.x, y + v.y);
    }

    /** Return the difference of this vector and another; does not alter either one. */
    public Vec2 sub(Vec2 v) {
        return new Vec2(x - v.x, y - v.y);
    }

    /** Return this vector multiplied by a scalar; does not alter this vector. */
    public Vec2 mul(float a) {
        return new Vec2(x * a, y * a);
    }

    /** Return the negation of this vector; does not alter this vector. */
    public Vec2 negate() {
        return new Vec2(-x, -y);
    }

    /** Flip the vector and return it - alters this vector. */
    public Vec2 negateLocal() {
        x = -x;
        y = -y;
        return this;
    }

    /** Add another vector to this one and return result - alters this vector. */
    public Vec2 addLocal(Vec2 v) {
        x += v.x;
        y += v.y;
        return this;
    }

    /** Subtract another vector from this one and return result - alters this vector. */
    public Vec2 subLocal(Vec2 v) {
        x -= v.x;
        y -= v.y;
        return this;
    }

    /** Multiply this vector by a number and return result - alters this vector. */
    public Vec2 mulLocal(float a) {
        x *= a;
        y *= a;
        return this;
    }

    /** Return the length of this vector. */
    public float length() {
        return (float) Math.sqrt(x * x + y * y);
    }
    
    /** Return the squared length of this vector. */
    public float lengthSquared() {
    	return (x*x + y*y);
    }

    /** Normalize this vector and return the length before normalization.  Alters this vector. */
    public float normalize() {
        float length = length();
        if (length < Settings.EPSILON) {
            return 0f;
        }

        float invLength = 1.0f / length;
        x *= invLength;
        y *= invLength;
        return length;
    }

    /** True if the vector represents a pair of valid, non-infinite floating point numbers. */ 
    public boolean isValid() {
        return x != Float.NaN && x != Float.NEGATIVE_INFINITY
                && x != Float.POSITIVE_INFINITY && y != Float.NaN
                && y != Float.NEGATIVE_INFINITY && y != Float.POSITIVE_INFINITY;
    }

    /** Return a new vector that has positive components. */
    public Vec2 abs() {
        return new Vec2(Math.abs(x), Math.abs(y));
    }

    @Override
    /** Return a copy of this vector. */
    public Vec2 clone() {
        return new Vec2(x, y);
    }

    @Override
    public String toString() {
        return "(" + x + "," + y + ")";
    }

    /*
     * Static
     */
    
    public static Vec2 abs(Vec2 a) {
    	return new Vec2(Math.abs(a.x), Math.abs(a.y));
    }
    
    public static float dot(Vec2 a, Vec2 b) {
        return a.x * b.x + a.y * b.y;
    }

    public static float cross(Vec2 a, Vec2 b) {
        return a.x * b.y - a.y * b.x;
    }

    public static Vec2 cross(Vec2 a, float s) {
        return new Vec2(s * a.y, -s * a.x);
    }

    public static Vec2 cross(float s, Vec2 a) {
        return new Vec2(-s * a.y, s * a.x);
    }

    public static Vec2 min(Vec2 a, Vec2 b) {
        return new Vec2(a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y);
    }

    public static Vec2 max(Vec2 a, Vec2 b) {
        return new Vec2(a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y);
    }
}
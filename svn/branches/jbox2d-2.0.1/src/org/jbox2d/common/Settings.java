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

//Updated to rev. 142 of b2Settings.cpp/.h

/** Global tuning constants based on MKS units and various integer maximums (vertices per shape, pairs, etc.). */
public class Settings {

    /** A "close to zero" float epsilon value for use */
    public static final float EPSILON = 1.1920928955078125E-7f;

    /** Pi. */
    public static final float pi = (float) Math.PI;
    
    // JBox2D specific settings
    /**
     * needs to be final, or will slow down math methods
     */
    public static final boolean FAST_MATH = true;
    public static final boolean SINCOS_LUT_ENABLED = true;
    /**
     * smaller the precision, the larger the table.  If
     * a small table is used (eg, precision is .006 or greater),
     * make sure you set the table to lerp it's results.  Accuracy chart
     * is in the MathUtils source.  Or, run the tests
     * yourself in {@link org.jbox2d.testbed.mathtests.SinCosTest}.</br>
     * </br>
     * Good lerp precision values:
     * <ul><li>.0092</li>
     * <li>.008201</li>
     * <li>.005904</li>
     * <li>.005204</li>
     * <li>.004305</li>
     * <li>.002807</li>
     * <li>.001508</li>
     * <li>9.32500E-4</li>
     * <li>7.48000E-4</li>
     * <li>8.47000E-4</li>
     * <li>.0005095</li>
     * <li>.0001098</li>
     * <li>9.50499E-5</li>
     * <li>6.08500E-5</li>
     * <li>3.07000E-5</li>
     * <li>1.53999E-5</li></ul>
     * 
     */
    public static final float SINCOS_LUT_PRECISION = .00131f;
	public static final int SINCOS_LUT_LENGTH = (int) Math.ceil(Math.PI*2 / SINCOS_LUT_PRECISION);
    /**
     * Use if the table's precision is large (eg .006 or greater).
     * Although it is more expensive, it greatly increases
     * accuracy.  Look in the MathUtils source for some test results
     * on the accuracy and speed of lerp vs non lerp.  Or, run the tests
     * yourself in {@link org.jbox2d.testbed.mathtests.SinCosTest}.
     */
    public static final boolean SINCOS_LUT_LERP = false;
    
    

    // Define your unit system here. The default system is
    // meters-kilograms-seconds. For the tuning to work well,
    // your dynamic objects should be bigger than a pebble and smaller
    // than a house.
    //
    // Use of these settings has been deprecated - they do not even
    // exist anymore in the C++ version of the engine, and future support
    // is unlikely.
    public static final float lengthUnitsPerMeter = 1.0f;
    public static final float massUnitsPerKilogram = 1.0f;
    public static final float timeUnitsPerSecond = 1.0f;

    // Collision
    
    public static final int maxManifoldPoints = 2;
    public static final int maxShapesPerBody = 64;
    public static final int maxPolygonVertices = 8;

    /** Must be a power of two. */
    public static final int maxProxies = 2048;
    /** Must be a power of two. */
    public static final int maxPairs = 8 * maxProxies;
    
    // Dynamics

    /**
     * A small length used as a collision and constraint tolerance. Usually it is
     * chosen to be numerically significant, but visually insignificant.
     */
    public static final float linearSlop = 0.005f * lengthUnitsPerMeter; // 0.5 cm

    /**
     * A small angle used as a collision and constraint tolerance. Usually it is
     * chosen to be numerically significant, but visually insignificant.
	 */
    public static final float angularSlop = 2.0f / 180.0f * pi; // 2 degrees

    /**
	 * A velocity threshold for elastic collisions. Any collision with a relative linear
     * velocity below this threshold will be treated as inelastic.
     */
    public static final float velocityThreshold = 1.0f * lengthUnitsPerMeter
            / timeUnitsPerSecond; // 1 m/s

    /**
     * The maximum linear position correction used when solving constraints. This helps to
     * prevent overshoot.
     */
    public static final float maxLinearCorrection = 0.2f * lengthUnitsPerMeter; // 20 cm

    /**
     * The maximum angular position correction used when solving constraints. This helps to
     * prevent overshoot.
     */
    public static final float maxAngularCorrection = 8.0f / 180.0f * pi; // 8 degrees

    /**
     * This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
     * that overlap is removed in one time step. However using values close to 1 often lead
     * to overshoot.
     */
    public static final float contactBaumgarte = 0.2f;

    /** The time that a body must be still before it will go to sleep. */
    public static final float timeToSleep = 0.5f * timeUnitsPerSecond; // half a second

    /** A body cannot sleep if its linear velocity is above this tolerance. */
    public static final float linearSleepTolerance = 0.01f
            * lengthUnitsPerMeter / timeUnitsPerSecond; // 1 cm/s

    /** A body cannot sleep if its angular velocity is above this tolerance. */
    public static final float angularSleepTolerance = 2.0f / 180.0f / timeUnitsPerSecond;
    
    /**
     * Continuous collision detection (CCD) works with core, shrunken shapes. This is the
     * amount by which shapes are automatically shrunk to work with CCD. This must be
     * larger than b2_linearSlop.
     */
    public static final float toiSlop = 8.0f * linearSlop;
    
    /**
     * The maximum linear velocity of a body. This limit is very large and is used
     * to prevent numerical problems. You shouldn't need to adjust this.
     */
    public static final float maxLinearVelocity = 200.0f;
    public static final float maxLinearVelocitySquared = maxLinearVelocity * maxLinearVelocity;

    /**
     * The maximum angular velocity of a body. This limit is very large and is used
     * to prevent numerical problems. You shouldn't need to adjust this.
     */
    public static final float maxAngularVelocity = 250.0f;
    public static final float maxAngularVelocitySquared = maxAngularVelocity * maxAngularVelocity;

    /** Maximum number of contacts to be handled to solve a TOI island. */
    public static int maxTOIContactsPerIsland = 32;
    
    /** Maximum number of joints to be handled to solve a TOI island. */
    public static int maxTOIJointsPerIsland = 0;//16;


}

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

//Updated to rev. 55 of b2Settings.cpp/.h

public class Settings {
    // Global tuning constants based on MKS units.

    // public static final float EPSILON = 0.0000000001f;

    /** A "close to zero" float epsilon value for use */
    public static final float EPSILON = 1.1920928955078125E-7f;

    public static final float pi = (float) Math.PI;

    // Define your unit system here. The default system is
    // meters-kilograms-seconds. For the tuning to work well,
    // your dynamic objects should be bigger than a pebble and smaller
    // than a house.
    public static final float lengthUnitsPerMeter = 1.0f;

    public static final float massUnitsPerKilogram = 1.0f;

    public static final float timeUnitsPerSecond = 1.0f;

    // Use this for pixels:
    // public static final float lengthUnitsPerMeter = 50.0f;

    // Global tuning constants based on MKS units.

    // Collision
    public static final int maxManifoldPoints = 2;

    public static final int maxShapesPerBody = 64;

    public static final int maxPolygonVertices = 8;

    public static final int maxProxies = 2048; // this must be a power of two

    public static final int maxPairs = 8 * maxProxies; // this must be a power of two
    
    // Dynamics
    
    public static final float linearSlop = 0.005f * lengthUnitsPerMeter; // 0.5 cm

    public static final float angularSlop = 2.0f / 180.0f * pi; // 2 degrees

    public static final float velocityThreshold = 1.0f * lengthUnitsPerMeter
            / timeUnitsPerSecond; // 1 m/s

    public static final float maxLinearCorrection = 0.2f * lengthUnitsPerMeter; // 20 cm

    public static final float maxAngularCorrection = 8.0f / 180.0f * pi; // 8 degrees

    public static final float contactBaumgarte = 0.2f;

    // Sleep
    public static final float timeToSleep = 0.5f * timeUnitsPerSecond; // half a second

    public static final float linearSleepTolerance = 0.01f
            * lengthUnitsPerMeter / timeUnitsPerSecond; // 1 cm/s

    // 2 degrees/s
    public static final float angularSleepTolerance = 2.0f / 180.0f / timeUnitsPerSecond;
    
    public static final float toiSlop = 8.0f * linearSlop;
    
  /// The maximum linear velocity of a body. This limit is very large and is used
  /// to prevent numerical problems. You shouldn't need to adjust this.
  public static final float maxLinearVelocity = 200.0f;
  public static final float maxLinearVelocitySquared = maxLinearVelocity * maxLinearVelocity;

  /// The maximum angular velocity of a body. This limit is very large and is used
  /// to prevent numerical problems. You shouldn't need to adjust this.
  public static final float maxAngularVelocity = 250.0f;
  public static final float maxAngularVelocitySquared = maxAngularVelocity * maxAngularVelocity;

  public static int maxTOIContactsPerIsland = 32;


}

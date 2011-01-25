/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DANIEL MURPHY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
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

/**
 * Global tuning constants based on MKS units and various integer maximums
 * (vertices per shape, pairs, etc.).
 */
public class Settings {
	
	/** A "close to zero" float epsilon value for use */
	public static final float EPSILON = 1.1920928955078125E-7f;
	
	/** Pi. */
	public static final float PI = (float) Math.PI;
	
	// JBox2D specific settings
	/**
	 * needs to be final, or will slow down math methods
	 */
	public static final boolean FAST_MATH = true;
	public static final boolean POOLING = true;
	public static final int CONTACT_STACK_INIT_SIZE = 10;
	public static final boolean SINCOS_LUT_ENABLED = false;
	/**
	 * smaller the precision, the larger the table. If a small table is used
	 * (eg, precision is .006 or greater), make sure you set the table to lerp
	 * it's results. Accuracy chart is in the MathUtils source. Or, run the
	 * tests yourself in {@link SinCosTest}.</br> </br> Good lerp precision
	 * values:
	 * <ul>
	 * <li>.0092</li>
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
	 * <li>1.53999E-5</li>
	 * </ul>
	 */
	public static final float SINCOS_LUT_PRECISION = .00011f;
	public static final int SINCOS_LUT_LENGTH = (int) Math.ceil(Math.PI * 2 / SINCOS_LUT_PRECISION);
	/**
	 * Use if the table's precision is large (eg .006 or greater). Although it
	 * is more expensive, it greatly increases accuracy. Look in the MathUtils
	 * source for some test results on the accuracy and speed of lerp vs non
	 * lerp. Or, run the tests yourself in {@link SinCosTest}.
	 */
	public static final boolean SINCOS_LUT_LERP = false;
	
	// Collision
	
	/**
	 * The maximum number of contact points between two convex shapes.
	 */
	public static int maxManifoldPoints = 2;
	
	/**
	 * The maximum number of vertices on a convex polygon.
	 */
	public static int maxPolygonVertices = 8;
	
	/**
	 * This is used to fatten AABBs in the dynamic tree. This allows proxies to
	 * move by a small amount without triggering a tree adjustment. This is in
	 * meters.
	 */
	public static float aabbExtension = 0.1f;
	
	/**
	 * This is used to fatten AABBs in the dynamic tree. This is used to predict
	 * the future position based on the current displacement.
	 * This is a dimensionless multiplier.
	 */
	public static float aabbMultiplier = 2.0f;
	
	/**
	 * A small length used as a collision and constraint tolerance. Usually it
	 * is chosen to be numerically significant, but visually insignificant.
	 */
	public static float linearSlop = 0.005f;
	
	/**
	 * A small angle used as a collision and constraint tolerance. Usually it is
	 * chosen to be numerically significant, but visually insignificant.
	 */
	public static float angularSlop = (2.0f / 180.0f * PI);
	
	/**
	 * The radius of the polygon/edge shape skin. This should not be modified.
	 * Making this smaller means polygons will have and insufficient for
	 * continuous collision. Making it larger may create artifacts for vertex
	 * collision.
	 */
	public static float polygonRadius = (2.0f * linearSlop);
	
	// Dynamics
	
	/**
	 * Maximum number of contacts to be handled to solve a TOI island.
	 */
	public static int maxTOIContacts = 32;
	
	/**
	 * A velocity threshold for elastic collisions. Any collision with a
	 * relative linear velocity below this threshold will be treated as
	 * inelastic.
	 */
	public static float velocityThreshold = 1.0f;
	
	/**
	 * The maximum linear position correction used when solving constraints.
	 * This helps to prevent overshoot.
	 */
	public static float maxLinearCorrection = 0.2f;
	
	/**
	 * The maximum angular position correction used when solving constraints.
	 * This helps to prevent overshoot.
	 */
	public static float maxAngularCorrection = (8.0f / 180.0f * PI);
	
	/**
	 * The maximum linear velocity of a body. This limit is very large and is
	 * used to prevent numerical problems. You shouldn't need to adjust this.
	 */
	public static float maxTranslation = 2.0f;
	public static float maxTranslationSquared = (maxTranslation * maxTranslation);
	
	/**
	 * The maximum angular velocity of a body. This limit is very large and is
	 * used to prevent numerical problems. You shouldn't need to adjust this.
	 */
	public static float maxRotation = (0.5f * PI);
	public static float maxRotationSquared = (maxRotation * maxRotation);
	
	/**
	 * This scale factor controls how fast overlap is resolved. Ideally this
	 * would be 1 so that overlap is removed in one time step. However using
	 * values close to 1 often lead to overshoot.
	 */
	public static float contactBaumgarte = 0.2f;
	
	// Sleep
	
	/**
	 * The time that a body must be still before it will go to sleep.
	 */
	public static float timeToSleep = 0.5f;
	
	/**
	 * A body cannot sleep if its linear velocity is above this tolerance.
	 */
	public static float linearSleepTolerance = 0.01f;
	
	/**
	 * A body cannot sleep if its angular velocity is above this tolerance.
	 */
	public static float angularSleepTolerance = (2.0f / 180.0f * PI);
	
	/**
	 * Friction mixing law. Feel free to customize this.
	 * TODO djm: add customization
	 * 
	 * @param friction1
	 * @param friction2
	 * @return
	 */
	public static final float mixFriction(float friction1, float friction2) {
		return MathUtils.sqrt(friction1 * friction2);
	}
	
	/**
	 * Restitution mixing law. Feel free to customize this.
	 * TODO djm: add customization
	 * 
	 * @param restitution1
	 * @param restitution2
	 * @return
	 */
	public static final float mixRestitution(float restitution1, float restitution2) {
		return restitution1 > restitution2 ? restitution1 : restitution2;
	}
}

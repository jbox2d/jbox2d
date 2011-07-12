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
package org.jbox2d.dynamics.contacts;

import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.Manifold.ManifoldType;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;

// updated to rev 100 - ec

public class ContactConstraint {
	public final ContactConstraintPoint points[];

 	public final Vec2 localNormal;
    public final Vec2 localPoint;
    public final Vec2 normal;
    
    public final Mat22 normalMass;
    public final Mat22 K;

    public Body bodyA;
    public Body bodyB;

    public ManifoldType type;
    
    public float radius;
    public float friction;
    public float restitution;
    public int pointCount;

    public Manifold manifold = null;

    public ContactConstraint() {
        points = new ContactConstraintPoint[Settings.maxManifoldPoints];
        for (int i = 0; i < Settings.maxManifoldPoints; i++) {
            points[i] = new ContactConstraintPoint();
        }
        pointCount = 0;
        localNormal = new Vec2();
        localPoint = new Vec2();
        normal = new Vec2();
        normalMass = new Mat22();
        K = new Mat22();

    }
    
    public void set(final ContactConstraint cp){
    	pointCount = cp.pointCount;
    	localNormal.set(cp.localNormal);
    	localPoint.set(cp.localPoint);
    	normal.set(cp.normal);
    	normalMass.set(cp.normalMass);
    	K.set(cp.K);
    	bodyA = cp.bodyA;
    	bodyB = cp.bodyB;
    	type = cp.type;
    	radius = cp.radius;
    	friction = cp.friction;
    	restitution = cp.restitution;
    	manifold = cp.manifold; // djm: not copy here
    	for(int i=0; i<cp.pointCount; i++){
    		points[i].set(cp.points[i]);
    	}
    }
}

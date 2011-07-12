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
package org.jbox2d.dynamics.joints;

import org.jbox2d.dynamics.Body;

/**
 * Definition for a {@link ConstantVolumeJoint}, which connects a group a bodies together
 * so they maintain a constant volume within them.
 */
public class ConstantVolumeJointDef extends JointDef {
	Body[] bodies;
	public float frequencyHz;
	public float dampingRatio;
	//public float relaxationFactor;//1.0 is perfectly stiff (but doesn't work, unstable)

	public ConstantVolumeJointDef() {
		type = JointType.CONSTANT_VOLUME;
		bodies = new Body[0];
		//relaxationFactor = 0.9f;
		collideConnected = false;
		frequencyHz = 0.0f;
		dampingRatio = 0.0f;
	}

	public void addBody(final Body b) {
		final Body[] tmp = new Body[bodies.length+1];
		System.arraycopy(bodies, 0, tmp, 0, bodies.length);
		tmp[bodies.length] = b;
		bodies = tmp;
		if (tmp.length == 1) {
			bodyA = b;
		}
		if (tmp.length == 2) {
			bodyB = b;
		}
	}
}

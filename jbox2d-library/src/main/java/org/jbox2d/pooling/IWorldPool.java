/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
package org.jbox2d.pooling;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.Collision;
import org.jbox2d.collision.Distance;
import org.jbox2d.collision.TimeOfImpact;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.Vec3;
import org.jbox2d.dynamics.contacts.Contact;

/**
 * World pool interface
 * @author Daniel
 *
 */
public interface IWorldPool {

	public IDynamicStack<Contact> getPolyContactStack();

	public IDynamicStack<Contact> getCircleContactStack();

	public IDynamicStack<Contact> getPolyCircleContactStack();

	public Vec2 popVec2();

	public Vec2[] popVec2(int argNum);

	public void pushVec2(int argNum);

	public Vec3 popVec3();

	public Vec3[] popVec3(int argNum);

	public void pushVec3(int argNum);

	public Mat22 popMat22();

	public Mat22[] popMat22(int argNum);

	public void pushMat22(int argNum);

	public AABB popAABB();

	public AABB[] popAABB(int argNum);

	public void pushAABB(int argNum);

	public Collision getCollision();

	public TimeOfImpact getTimeOfImpact();

	public Distance getDistance();

	public float[] getFloatArray(int argLength);

	public int[] getIntArray(int argLength);

	public Vec2[] getVec2Array(int argLength);

}

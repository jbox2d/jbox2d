/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

//#ifndef B2_WORLD_CALLBACKS_H
//#define B2_WORLD_CALLBACKS_H

//struct b2Joint;

// If a body is destroyed, then any joints attached to it are also destroyed.
// This prevents memory leaks, but you may unexpectedly be left with an
// orphaned pointer.
// This callback class will notify you when a joint is implicitly destroyed.
// It is NOT called if you directly destroy a joint.
// Implement this abstract class and provide it to b2World via
// b2World::SetJointDestroyedCallback(). 
abstract class b2JointDestroyedCallback{
	public void b2JointDestroyedCallbackDestructor() {}

	// This function is called right before the joint is destroyed. When this
	// function is called, you should remove any reference you may have to this joint.
	public abstract void Notify(b2Joint joint);
};

//#endif

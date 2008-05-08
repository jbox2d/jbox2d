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

package org.jbox2d.collision;

// Updated to rev 139 of b2Shape.h

/** This holds contact filtering data. */
public class FilterData {
	/** The collision category bits. Normally you would just set one bit. */
	public int categoryBits;

	/**
	 * The collision mask bits. This states the categories that this
	 * shape would accept for collision.
	 */
	public int maskBits;

	/**
	 * Collision groups allow a certain group of objects to never collide (negative)
	 * or always collide (positive). Zero means no collision group. Non-zero group
	 * filtering always wins against the mask bits.
	 */
	public int groupIndex;
	
	public void set(FilterData fd) {
		categoryBits = fd.categoryBits;
		maskBits = fd.maskBits;
		groupIndex = fd.groupIndex;
	}
}

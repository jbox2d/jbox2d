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

import org.jbox2d.common.Mat22;
import org.jbox2d.common.Vec2;

//Updated to rev 56 of b2Collision.h

public class OBB {
    public Mat22 R;

    public Vec2 center;

    public Vec2 extents;

    public OBB(Mat22 _R, Vec2 _center, Vec2 _extents) {
        R = _R.clone();
        center = _center.clone();
        extents = _extents.clone();
    }

    public OBB(OBB copy) {
        this(copy.R.clone(), copy.center.clone(), copy.extents.clone());
    }

    public OBB() {
        R = new Mat22();
        center = new Vec2();
        extents = new Vec2();
    }

    public OBB clone() {
        return new OBB(this);
    }

}
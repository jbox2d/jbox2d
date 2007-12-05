/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.gphysics.com
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
package collision;

import common.Vec2;

//Updated to rev 56 of b2Collision.h

public class AABB {
    public Vec2 minVertex, maxVertex;

    public AABB(Vec2 minVertex, Vec2 maxVertex) {
        this.minVertex = minVertex.clone(); // clone to be safe
        this.maxVertex = maxVertex.clone();
    }

    public AABB(AABB copy) {
        this(copy.minVertex.clone(), copy.maxVertex.clone());
    }

    public AABB() {
        minVertex = new Vec2();
        maxVertex = new Vec2();
    }

    boolean isValid() {
        Vec2 d = maxVertex.sub(minVertex);
        return d.x >= 0.0f && d.y >= 0 && minVertex.isValid()
                && maxVertex.isValid();
    }

    boolean testOverlap(AABB box) {
        Vec2 d1 = box.minVertex.sub(maxVertex);
        Vec2 d2 = minVertex.sub(box.maxVertex);

        if (d1.x > 0.0f || d1.y > 0.0f || d2.x > 0.0f || d2.y > 0.0f) {
            return false;
        }
        else {
            return true;
        }
    }
}

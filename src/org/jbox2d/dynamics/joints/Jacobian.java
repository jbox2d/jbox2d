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

package org.jbox2d.dynamics.joints;

import org.jbox2d.common.Vec2;

public class Jacobian {
    public Vec2 linear1;

    public float angular1;

    public Vec2 linear2;

    public float angular2;

    public Jacobian() {
        linear1 = new Vec2();
        linear2 = new Vec2();
        setZero();
    }

    void setZero() {
        linear1.setZero();
        angular1 = 0.0f;
        linear2.setZero();
        angular2 = 0.0f;
    }

    void set(Vec2 x1, float a1, Vec2 x2, float a2) {
        linear1 = x1;
        angular1 = a1;
        linear2 = x2;
        angular2 = a2;
    }

    float compute(Vec2 x1, float a1, Vec2 x2, float a2) {
        return Vec2.dot(linear1, x1) + angular1 * a1 + Vec2.dot(linear2, x2)
                + angular2 * a2;
    }
}

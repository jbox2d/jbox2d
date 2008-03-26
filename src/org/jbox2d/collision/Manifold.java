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

import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;

//Updated to rev 56->108 of b2Collision.h

public class Manifold {
    public ManifoldPoint[] points;

    public Vec2 normal;

    public int pointCount;

    public Manifold() {
        points = new ManifoldPoint[Settings.maxManifoldPoints];
        for (int i = 0; i < Settings.maxManifoldPoints; i++) {
            points[i] = new ManifoldPoint();
        }
        normal = new Vec2();
        pointCount = 0;
    }

    public Manifold(Manifold other) {
        points = new ManifoldPoint[Settings.maxManifoldPoints];
        // FIXME? Need to check how C++ handles an implicit
        // copy of a Manifold, by copying the points array
        // or merely passing a pointer to it.
        // Update: tested both, the arraycopy seems to be correct, leave it!
        System.arraycopy(other.points, 0, points, 0, other.points.length);
        // points = new ManifoldPoint[other.points.length];
        // for (int i=0; i<other.pointCount; i++){
        //   points[i] = new ManifoldPoint(other.points[i]);
        // }
        normal = other.normal.clone();
        pointCount = other.pointCount;// points.length;
    }
}

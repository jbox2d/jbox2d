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

public class Proxy {
    public int lowerBounds[];

    public int upperBounds[];

    int overlapCount;

    int timeStamp;

    int categoryBits;

    int maskBits;

    int groupIndex;

    Object userData;

    public Proxy() {
        lowerBounds = new int[2];
        upperBounds = new int[2];
        lowerBounds[0] = lowerBounds[1] = 0;
        upperBounds[0] = upperBounds[1] = 0;
        overlapCount = BroadPhase.INVALID;
        timeStamp = 0;
    }

    int getNext() {
        return lowerBounds[0];
    }

    void setNext(int next) {
        lowerBounds[0] = next;
    }

    public boolean isValid() {
        return overlapCount != BroadPhase.INVALID;
    }
}

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

//Updated to rev 56 of b2PairManager.h

public class BufferedPair implements Comparable<BufferedPair> {
    int proxyId1;

    int proxyId2;

    private boolean equals(BufferedPair other) {
        return proxyId1 == other.proxyId1 && proxyId2 == other.proxyId2;
    }

    private boolean minor(BufferedPair other) {
        if (proxyId1 < other.proxyId1)
            return true;

        if (proxyId1 == other.proxyId1) {
            return proxyId2 < other.proxyId2;
        }

        return false;
    }

    public int compareTo(BufferedPair p) {
        if (minor(p)) {
            return -1;
        }
        else if (equals(p)) {
            return 0;
        }
        else {
            return 1;
        }
    }
}

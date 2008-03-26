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

public class Pair implements Comparable<Pair> {
    private static final int PAIR_BUFFERED = 0x0001;

    private static final int PAIR_REMOVED = 0x0002;

    private static final int PAIR_FINAL = 0x0004;

    public Object userData;

    public int proxyId1;

    public int proxyId2;

    public int status;
    
    public int next;

    public Pair() {

    }

    /**
     * Copy constructor
     */
    public Pair(Pair other) {
        this.userData = other.userData;
        this.proxyId1 = other.proxyId1;
        this.proxyId2 = other.proxyId2;
        this.status = other.status;
    }

    public void setBuffered() {
        status |= PAIR_BUFFERED;
    }

    public void clearBuffered() {
        status &= ~PAIR_BUFFERED;
    }

    public boolean isBuffered() {
        return (status & PAIR_BUFFERED) == PAIR_BUFFERED;
    }

    public void clearRemoved() {
        status &= ~PAIR_REMOVED;
    }

    public void setRemoved() {
        status |= PAIR_REMOVED;
    }

    public boolean isRemoved() {
        return (status & PAIR_REMOVED) == PAIR_REMOVED;
    }

    public void setFinal() {
        status |= PAIR_FINAL;
    }

    public boolean isFinal() {
        return (status & PAIR_FINAL) == PAIR_FINAL;
    }

    public int compareTo(Pair p) {
        // XXX check
        return proxyId1 - p.proxyId1;
    }
}
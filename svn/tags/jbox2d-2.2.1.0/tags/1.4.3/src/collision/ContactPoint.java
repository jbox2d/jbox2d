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

public class ContactPoint {
    public Vec2 position;

    public float separation;

    public float normalImpulse;

    public float tangentImpulse;

    public ContactID id;

    public ContactPoint() {
        position = new Vec2();
        separation = normalImpulse = tangentImpulse = 0f;
        id = new ContactID();
    }

    public ContactPoint(ContactPoint cp) {
        position = cp.position.clone();
        separation = cp.separation;
        normalImpulse = cp.normalImpulse;
        tangentImpulse = cp.tangentImpulse;
        id = new ContactID(cp.id);
    }
}

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
package dynamics;

//Updated to rev. 56 of b2Body.h

import collision.ShapeDef;

import common.Settings;
import common.Vec2;

public class BodyDef {

    public Object userData;

    public ShapeDef[] shapes;

    public Vec2 position;

    public float rotation;

    public Vec2 linearVelocity;

    public float angularVelocity;

    public float linearDamping;

    public float angularDamping;

    public boolean allowSleep;

    public boolean isSleeping;

    public boolean preventRotation;

    public BodyDef() {
        userData = null;
        shapes = new ShapeDef[Settings.maxShapesPerBody];
        position = new Vec2(0.0f, 0.0f);
        rotation = 0.0f;
        linearVelocity = new Vec2(0.0f, 0.0f);
        angularVelocity = 0.0f;
        linearDamping = 0.0f;
        angularDamping = 0.0f;
        allowSleep = true;
        isSleeping = false;
        preventRotation = false;
    }

    public void addShape(ShapeDef shape) {
        for (int i = 0; i < Settings.maxShapesPerBody; ++i) {
            if (shapes[i] == null) {
                shapes[i] = shape;
                // System.out.println(shape.type);
                break;
            }
        }
    }
}

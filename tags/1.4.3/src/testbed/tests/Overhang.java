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
package testbed.tests;

import processing.core.PApplet;
import testbed.PTest;
import collision.BoxDef;

import common.Vec2;

import dynamics.BodyDef;
import dynamics.World;

public class Overhang extends PTest {

    public Overhang() {
        super("Overhang");
    }

    @Override
    public void go(World world) {
        {
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(50.0f, 10.0f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f, -10.0f);
            bd.addShape(sd);
            world.createBody(bd);
        }

        {
            BoxDef sd = new BoxDef();
            float w = 4.0f;
            float h = 0.25f;
            sd.extents = new Vec2(w, h);
            sd.density = 1.0f;
            sd.friction = 0.3f;
            sd.restitution = 0.0f;

            BodyDef bd = new BodyDef();
            bd.addShape(sd);

            int numSlats = 8;
            float lastCMX = 0.0f;
            float eps = 0.14f;
            for (int i = 0; i < numSlats; ++i) {
                float newX = lastCMX + w - eps;
                lastCMX = (i * lastCMX + newX) / (i + 1);
                bd.position = new Vec2(newX, .25f + 2 * h * (numSlats - i - 1));
                m_world.createBody(bd);
            }

        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[] { "testbed.tests.Overhang" });
    }
}
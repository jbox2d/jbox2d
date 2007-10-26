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

import java.util.Random;

import processing.core.PApplet;

import testbed.PTest;
import collision.AABB;
import collision.BoxDef;

import common.Vec2;

import dynamics.BodyDef;
import dynamics.World;

public class CompoundShapes extends PTest {

    public CompoundShapes() {
        super("CompoundShapes");
    }

    @Override
    public void setupWorld() {
        m_world = new World(new AABB(new Vec2(-100f, -100f), new Vec2(100f,
                200f)), new Vec2(0.0f, -10.0f), true);
        m_bomb = null;
        m_mouseJoint = null;
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
            BoxDef sd1 = new BoxDef();
            sd1.extents = new Vec2(0.25f, 0.5f);
            // sd1.localPosition=new Vec2(-0.5f, 0.0f);
            sd1.density = 1.0f;

            BoxDef sd2 = new BoxDef();
            sd2.extents = new Vec2(0.25f, 0.5f);
            // sd2.localPosition=new Vec2(0.5f, 0.0f);
            sd2.localPosition = new Vec2(0.0f, -0.5f);
            sd2.localRotation = (float) (0.5f * Math.PI);
            sd2.density = 1.0f;

            BodyDef bd = new BodyDef();
            bd.addShape(sd1);
            bd.addShape(sd2);

            Random r = new Random();

            for (int i = 0; i < 100; ++i) {
                float x = r.nextFloat() * 0.2f - 0.1f;
                bd.position = new Vec2(x, 1.05f + 1.5f * i);
                // bd.position=new Vec2(0.0f, 0.45f);
                bd.rotation = (float) (r.nextFloat() * Math.PI * 4 - Math.PI * 2);

                world.createBody(bd);
            }
        }
    }

    /**
     * Entry point
     */
    public static void main(String[] argv) {
        PApplet.main(new String[] { "testbed.tests.CompoundShapes" });
    }
}

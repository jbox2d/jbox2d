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
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.PolygonDef;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;

public class Overhang extends AbstractExample {

    public Overhang(TestbedMain p) {
        super(p);
    }
    
    public String getName() {
    	return "Overhang";
    }

    @Override
    public void create() {
        {
            PolygonDef sd = new PolygonDef();
            sd.setAsBox(50.0f, 10.0f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f, -10.0f);
            m_world.createStaticBody(bd).createShape(sd);
        }

        {
            PolygonDef sd = new PolygonDef();
            float w = 4.0f;
            float h = 0.25f;
            sd.setAsBox(w, h);
            sd.density = 1.0f;
            sd.friction = 0.3f;
            sd.restitution = 0.0f;

            BodyDef bd = new BodyDef();

            int numSlats = 8;
            float lastCMX = 0.0f;
            float eps = 0.14f;
            for (int i = 0; i < numSlats; ++i) {
                float newX = lastCMX + w - eps;
                lastCMX = (i * lastCMX + newX) / (i + 1);
                bd.position = new Vec2(newX, .25f + 2 * h * (numSlats - i - 1));
                Body myBody = m_world.createDynamicBody(bd);
                myBody.createShape(sd);
                myBody.setMassFromShapes();
            }

        }
    }

}
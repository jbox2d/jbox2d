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

import java.util.Random;

import org.jbox2d.collision.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;

import processing.core.PApplet;


public class CompoundShapes extends AbstractExample {

	private boolean firstTime = true;
	private final int stackHeight = 100;
	
	public CompoundShapes(TestbedMain _parent) {
		super(_parent);
	}
	
	// Default world AABB is too small, make a bigger one
	public void createWorld() {
		m_worldAABB = new AABB();
		m_worldAABB.lowerBound = new Vec2(-200.0f, -100.0f);
		m_worldAABB.upperBound = new Vec2(200.0f, 500.0f);
		Vec2 gravity = new Vec2(0.0f, -10.0f);
		boolean doSleep = true;
		m_world = new World(m_worldAABB, gravity, doSleep);
	}

    public void create() {
    	if (firstTime) {
			setCamera(0f, 10f, 15f);
			firstTime = false;
		}
    	
    	{
			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, -10.0f);
			Body body = m_world.createStaticBody(bd);

			PolygonDef sd = new PolygonDef();;
			sd.setAsBox(50.0f, 10.0f);
			body.createShape(sd);
		}

		{
			CircleDef sd1 = new CircleDef();
			sd1.radius = 0.5f;
			sd1.localPosition.set(-0.5f, 0.5f);
			sd1.density = 2.0f;

			CircleDef sd2 = new CircleDef();
			sd2.radius = 0.5f;
			sd2.localPosition.set(0.5f, 0.5f);
			sd2.density = 0.0f; // massless

			for (int i = 0; i < stackHeight; ++i)
			{
				float x = parent.random(-0.1f, 0.1f);
				BodyDef bd = new BodyDef();
				bd.position.set(x + 5.0f, 1.05f + 2.5f * i);
				bd.angle = parent.random(-3.1415f, 3.1415f);
				Body body = m_world.createDynamicBody(bd);
				body.createShape(sd1);
				body.createShape(sd2);
				body.setMassFromShapes();
			}
		}

		{
			PolygonDef sd1 = new PolygonDef();
			sd1.setAsBox(0.25f, 0.5f);
			sd1.density = 2.0f;

			PolygonDef sd2 = new PolygonDef();;
			sd2.setAsBox(0.25f, 0.5f, new Vec2(0.0f, -0.5f), 0.5f * 3.1415f);
			sd2.density = 2.0f;

			for (int i = 0; i < stackHeight; ++i)
			{
				float x = parent.random(-0.1f, 0.1f);
				BodyDef bd = new BodyDef();
				bd.position.set(x - 5.0f, 1.05f + 2.5f * i);
				bd.angle = parent.random(-3.1415f, 3.1415f);
				Body body = m_world.createDynamicBody(bd);
				body.createShape(sd1);
				body.createShape(sd2);
				body.setMassFromShapes();
			}
		}

		{
			XForm xf1 = new XForm();
			xf1.R.set(0.3524f * 3.1415f);
			xf1.position = Mat22.mul(xf1.R, new Vec2(1.0f, 0.0f));

			PolygonDef sd1 = new PolygonDef();
			sd1.vertices.add(XForm.mul(xf1, new Vec2(-1.0f, 0.0f)));
			sd1.vertices.add(XForm.mul(xf1, new Vec2(1.0f, 0.0f)));
			sd1.vertices.add(XForm.mul(xf1, new Vec2(0.0f, 0.5f)));
			sd1.density = 2.0f;

			XForm xf2 = new XForm();
			xf2.R.set(-0.3524f * 3.1415f);
			xf2.position = Mat22.mul(xf2.R, new Vec2(-1.0f, 0.0f));

			PolygonDef sd2 = new PolygonDef();
			sd2.vertices.add(XForm.mul(xf2, new Vec2(-1.0f, 0.0f)));
			sd2.vertices.add(XForm.mul(xf2, new Vec2(1.0f, 0.0f)));
			sd2.vertices.add(XForm.mul(xf2, new Vec2(0.0f, 0.5f)));
			sd2.density = 2.0f;

			for (int i = 0; i < stackHeight; ++i)
			{
				float x = parent.random(-0.1f, 0.1f);
				BodyDef bd = new BodyDef();
				bd.position.set(x, 2.05f + 2.5f * i);
				bd.angle = 0.0f;
				Body body = m_world.createDynamicBody(bd);
				body.createShape(sd1);
				body.createShape(sd2);
				body.setMassFromShapes();
			}
		}

		{
			PolygonDef sd_bottom = new PolygonDef();
			sd_bottom.setAsBox( 1.5f, 0.15f );
			sd_bottom.density = 4.0f;

			PolygonDef sd_left = new PolygonDef();
			sd_left.setAsBox(0.15f, 2.7f, new Vec2(-1.45f, 2.35f), 0.2f);
			sd_left.density = 4.0f;

			PolygonDef sd_right = new PolygonDef();
			sd_right.setAsBox(0.15f, 2.7f, new Vec2(1.45f, 2.35f), -0.2f);
			sd_right.density = 4.0f;

			BodyDef bd = new BodyDef();
			bd.position.set( 0.0f, 2.0f );
			Body body = m_world.createDynamicBody(bd);
			body.createShape(sd_bottom);
			body.createShape(sd_left);
			body.createShape(sd_right);
			body.setMassFromShapes();
		}
    }
    
    public String getName() {
    	return "Compound Shapes";
    }
}

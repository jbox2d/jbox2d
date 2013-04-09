/*******************************************************************************
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
/**
 * Created at 2:15:39 PM Jan 23, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.PrismaticJointDef;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class CollisionFiltering extends TestbedTest {
	
	// This is a test of collision filtering.
	// There is a triangle, a box, and a circle.
	// There are 6 shapes. 3 large and 3 small.
	// The 3 small ones always collide.
	// The 3 large ones never collide.
	// The boxes don't collide with triangles (except if both are small).
	final int k_smallGroup = 1;
	final int k_largeGroup = -1;

	final int k_defaultCategory = 0x0001;
	final int k_triangleCategory = 0x0002;
	final int k_boxCategory = 0x0004;
	final int k_circleCategory = 0x0008;

	final int k_triangleMask = 0xFFFF;
	final int k_boxMask = 0xFFFF ^ k_triangleCategory;
	final int k_circleMask = 0xFFFF;
	
	@Override
	public boolean isSaveLoadEnabled() {
	  return true;
	}

	@Override
	public void initTest(boolean deserialized) {
	  if(deserialized){
	    return;
	  }
		// Ground body
		{
			EdgeShape shape = new EdgeShape();
			shape.set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));

			FixtureDef sd = new FixtureDef();
			sd.shape = shape;
			sd.friction = 0.3f;

			BodyDef bd = new BodyDef();
			Body ground = getWorld().createBody(bd);
			ground.createFixture(sd);
		}

		// Small triangle
		Vec2 vertices[] = new Vec2[3];
		vertices[0] = new Vec2(-1.0f, 0.0f);
		vertices[1] = new Vec2(1.0f, 0.0f);
		vertices[2] = new Vec2(0.0f, 2.0f);
		PolygonShape polygon = new PolygonShape();
		polygon.set(vertices, 3);

		FixtureDef triangleShapeDef = new FixtureDef();
		triangleShapeDef.shape = polygon;
		triangleShapeDef.density = 1.0f;

		triangleShapeDef.filter.groupIndex = k_smallGroup;
		triangleShapeDef.filter.categoryBits = k_triangleCategory;
		triangleShapeDef.filter.maskBits = k_triangleMask;

		BodyDef triangleBodyDef = new BodyDef();
		triangleBodyDef.type = BodyType.DYNAMIC;
		triangleBodyDef.position.set(-5.0f, 2.0f);

		Body body1 = getWorld().createBody(triangleBodyDef);
		body1.createFixture(triangleShapeDef);

		// Large triangle (recycle definitions)
		vertices[0].mulLocal(2.0f);
		vertices[1].mulLocal(2.0f);
		vertices[2].mulLocal(2.0f);
		polygon.set(vertices, 3);
		triangleShapeDef.filter.groupIndex = k_largeGroup;
		triangleBodyDef.position.set(-5.0f, 6.0f);
		triangleBodyDef.fixedRotation = true; // look at me!

		Body body2 = getWorld().createBody(triangleBodyDef);
		body2.createFixture(triangleShapeDef);

		{
			BodyDef bd = new BodyDef();
			bd.type = BodyType.DYNAMIC;
			bd.position.set(-5.0f, 10.0f);
			Body body = getWorld().createBody(bd);

			PolygonShape p = new PolygonShape();
			p.setAsBox(0.5f, 1.0f);
			body.createFixture(p, 1.0f);

			PrismaticJointDef jd = new PrismaticJointDef();
			jd.bodyA = body2;
			jd.bodyB = body;
			jd.enableLimit = true;
			jd.localAnchorA.set(0.0f, 4.0f);
			jd.localAnchorB.setZero();
			jd.localAxisA.set(0.0f, 1.0f);
			jd.lowerTranslation = -1.0f;
			jd.upperTranslation = 1.0f;

			getWorld().createJoint(jd);
		}

		// Small box
		polygon.setAsBox(1.0f, 0.5f);
		FixtureDef boxShapeDef = new FixtureDef();
		boxShapeDef.shape = polygon;
		boxShapeDef.density = 1.0f;
		boxShapeDef.restitution = 0.1f;

		boxShapeDef.filter.groupIndex = k_smallGroup;
		boxShapeDef.filter.categoryBits = k_boxCategory;
		boxShapeDef.filter.maskBits = k_boxMask;

		BodyDef boxBodyDef = new BodyDef();
		boxBodyDef.type = BodyType.DYNAMIC;
		boxBodyDef.position.set(0.0f, 2.0f);

		Body body3 = getWorld().createBody(boxBodyDef);
		body3.createFixture(boxShapeDef);

		// Large box (recycle definitions)
		polygon.setAsBox(2.0f, 1.0f);
		boxShapeDef.filter.groupIndex = k_largeGroup;
		boxBodyDef.position.set(0.0f, 6.0f);

		Body body4 = getWorld().createBody(boxBodyDef);
		body4.createFixture(boxShapeDef);

		// Small circle
		CircleShape circle = new CircleShape();
		circle.m_radius = 1.0f;

		FixtureDef circleShapeDef = new FixtureDef();
		circleShapeDef.shape = circle;
		circleShapeDef.density = 1.0f;

		circleShapeDef.filter.groupIndex = k_smallGroup;
		circleShapeDef.filter.categoryBits = k_circleCategory;
		circleShapeDef.filter.maskBits = k_circleMask;

		BodyDef circleBodyDef = new BodyDef();
		circleBodyDef.type = BodyType.DYNAMIC;
		circleBodyDef.position.set(5.0f, 2.0f);
		
		Body body5 = getWorld().createBody(circleBodyDef);
		body5.createFixture(circleShapeDef);

		// Large circle
		circle.m_radius *= 2.0f;
		circleShapeDef.filter.groupIndex = k_largeGroup;
		circleBodyDef.position.set(5.0f, 6.0f);

		Body body6 = getWorld().createBody(circleBodyDef);
		body6.createFixture(circleShapeDef);
	}

	@Override
	public String getTestName() {
		return "Collision Filtering";
	}
	
}

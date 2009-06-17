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

package org.jbox2d.testbed.tests;

import java.util.ArrayList;

import org.jbox2d.collision.shapes.CircleDef;
import org.jbox2d.collision.shapes.PolygonDef;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.joints.ConstantVolumeJoint;
import org.jbox2d.dynamics.joints.ConstantVolumeJointDef;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;
import processing.core.PApplet;

public class BlobTest7 extends AbstractExample {
	private boolean firstTime;
	
	public BlobTest7(TestbedMain _parent) {
		super(_parent);
		firstTime = true;
	}
	
	@Override
	public void create() {

		if (firstTime) {
			setCamera(0.0f,10.0f,20.0f);
			firstTime = false;
			//this.settings.drawJoints = false;
		}
		
    	Body ground = null;
		{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(50.0f, 0.4f);

			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, 0.0f);
			ground = m_world.createBody(bd);
			ground.createShape(sd);
			
			sd.setAsBox(0.4f,50.0f,new Vec2(-10.0f,0.0f), 0.0f);
			ground.createShape(sd);
			sd.setAsBox(0.4f,50.0f,new Vec2(10.0f,0.0f), 0.0f);
			ground.createShape(sd);
		}
		
		ConstantVolumeJointDef cvjd = new ConstantVolumeJointDef();
		
		float cx = 0.0f;
		float cy = 10.0f;
		float rx = 4.0f;
		float ry = 4.0f;
		int nBodies = 100;
		float bodyRadius = 0.4f;
		ArrayList<Body> bodies = new ArrayList<Body>();
		for (int i=0; i<nBodies; ++i) {
			float angle = PApplet.map(i, 0, nBodies, 0, 2*3.1415f);
			BodyDef bd = new BodyDef();
			//bd.isBullet = true;
			bd.fixedRotation = true;
		
			float x = cx + (1.0f + .5f*(float)Math.cos(4*(angle+.25f*3.1415f))) * rx * (float)Math.sin(angle);
			float y = cy + (1.0f + .5f*(float)Math.cos(4*(angle+.25f*3.1415f))) * ry * (float)Math.cos(angle);
			bd.position.set(new Vec2(x,y));
			Body body = m_world.createBody(bd);
			CircleDef cd = new CircleDef();
			cd.radius = bodyRadius;
			cd.density = 1.0f;
			//cd.filter.groupIndex = -2;
			body.createShape(cd);
			cvjd.addBody(body);
			body.setMassFromShapes();
			bodies.add(body);
		}

		cvjd.frequencyHz = 10.0f;
		cvjd.dampingRatio = 10.0f;
		((ConstantVolumeJoint)m_world.createJoint(cvjd)).inflate(1.5f);
		
		cvjd = new ConstantVolumeJointDef();
		for (int i=0; i<nBodies/4; ++i) {
			cvjd.addBody(bodies.get(i));
		}
		cvjd.frequencyHz = 10.0f;
		cvjd.dampingRatio = 10.0f;
		((ConstantVolumeJoint)m_world.createJoint(cvjd)).inflate(1.5f);
		
		cvjd = new ConstantVolumeJointDef();
		for (int i=nBodies/4; i<nBodies/2; ++i) {
			cvjd.addBody(bodies.get(i));
		}
		cvjd.frequencyHz = 10.0f;
		cvjd.dampingRatio = 10.0f;
		((ConstantVolumeJoint)m_world.createJoint(cvjd)).inflate(1.5f);
		
		cvjd = new ConstantVolumeJointDef();
		for (int i=nBodies/2; i<3*nBodies/4; ++i) {
			cvjd.addBody(bodies.get(i));
		}

		cvjd.frequencyHz = 10.0f;
		cvjd.dampingRatio = 10.0f;
		((ConstantVolumeJoint)m_world.createJoint(cvjd)).inflate(1.5f);
		
		cvjd = new ConstantVolumeJointDef();
		for (int i=3*nBodies/4; i<nBodies; ++i) {
			cvjd.addBody(bodies.get(i));
		}

		cvjd.frequencyHz = 10.0f;
		cvjd.dampingRatio = 10.0f;
		((ConstantVolumeJoint)m_world.createJoint(cvjd)).inflate(1.5f);
		
//		cvjd = new ConstantVolumeJointDef();
//		for (int i=0+nBodies/8; i<nBodies/4+nBodies/8; ++i) {
//			cvjd.addBody(bodies.get(i));
//		}
//		cvjd.frequencyHz = 10.0f;
//		cvjd.dampingRatio = 10.0f;
//		m_world.createJoint(cvjd);
//		
//		cvjd = new ConstantVolumeJointDef();
//		for (int i=nBodies/4+nBodies/8; i<nBodies/2+nBodies/8; ++i) {
//			cvjd.addBody(bodies.get(i));
//		}
//		cvjd.frequencyHz = 10.0f;
//		cvjd.dampingRatio = 10.0f;
//		m_world.createJoint(cvjd);
//		
//		cvjd = new ConstantVolumeJointDef();
//		for (int i=nBodies/2+nBodies/8; i<3*nBodies/4+nBodies/8; ++i) {
//			cvjd.addBody(bodies.get(i));
//		}
//		cvjd.frequencyHz = 10.0f;
//		cvjd.dampingRatio = 10.0f;
//		m_world.createJoint(cvjd);
//		cvjd = new ConstantVolumeJointDef();
//		for (int i=3*nBodies/4+nBodies/8; i<nBodies+nBodies/8; ++i) {
//			int index = i;
//			if (index >= bodies.size()) index -= bodies.size();
//			cvjd.addBody(bodies.get(index));
//		}
//
//		cvjd.frequencyHz = 10.0f;
//		cvjd.dampingRatio = 10.0f;
//		m_world.createJoint(cvjd);
		
//		cvjd = new ConstantVolumeJointDef();
//		cvjd.addBody(bodies.get(0));
//		cvjd.addBody(bodies.get(nBodies/4));
//		cvjd.addBody(bodies.get(nBodies/2));
//		cvjd.addBody(bodies.get(3*nBodies/4));
//		cvjd.frequencyHz = 10.0f;
//		m_world.createJoint(cvjd);
//		
//		cvjd = new ConstantVolumeJointDef();
//		cvjd.addBody(bodies.get(0 + nBodies/8));
//		cvjd.addBody(bodies.get(nBodies/4 + nBodies/8));
//		cvjd.addBody(bodies.get(nBodies/2 + nBodies/8));
//		cvjd.addBody(bodies.get(3*nBodies/4 + nBodies/8));
//		cvjd.frequencyHz = 1.0f;
//		m_world.createJoint(cvjd);
		
		BodyDef bd2 = new BodyDef();
		PolygonDef psd = new PolygonDef();
		psd.setAsBox(3.0f,1.5f,new Vec2(cx,cy+15.0f),0.0f);
		psd.density = 1.0f;
		bd2.position = new Vec2(cx,cy+15.0f);
		Body fallingBox = m_world.createBody(bd2);
		fallingBox.createShape(psd);
		fallingBox.setMassFromShapes();
	}

	@Override
	public String getName() {
		return "Blob Joint Test";
	}

}
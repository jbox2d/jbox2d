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

import java.awt.event.InputEvent;
import java.util.ArrayList;

import org.jbox2d.callbacks.QueryCallback;
import org.jbox2d.collision.AABB;
import org.jbox2d.collision.Distance;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.ConstantVolumeJointDef;
import org.jbox2d.dynamics.joints.DistanceJointDef;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.JointDef;
import org.jbox2d.dynamics.joints.JointType;
import org.jbox2d.dynamics.joints.MouseJointDef;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.dynamics.joints.RopeJointDef;
import org.jbox2d.dynamics.joints.WheelJointDef;
import org.jbox2d.testbed.framework.TestbedTest;
import org.jbox2d.testbed.framework.j2d.TestbedMain;
import org.jbox2d.testbed.framework.j2d.TestbedSidePanel;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class VertexTest extends TestbedTest {
	private static final Logger log = LoggerFactory.getLogger(TestbedTest.class);
	ArrayList<Body> g_vertecies;

	private boolean m_isCreatingEdge = false;
	private Vec2 mFrom;
	private Vec2 mTo;
	private Body selectedBody; 
	private Body recentBody; 
	
	
  public VertexTest()
  {
	  g_vertecies = new ArrayList<Body>();
  }
	
  @Override
  public float getDefaultCameraScale() {
    return 20;
  }

  @Override
  public boolean isSaveLoadEnabled() {
    return true;
  }

  @Override
  public void initTest(boolean deserialized) {
    if (deserialized) {
      return;
    }

    Body ground = null;
    {
      PolygonShape sd = new PolygonShape();
      //sd.setAsBox(50.0f, 0.01f, new Vec2(0.0f, -5.0f), 0.0f);
      sd.setAsBox(50.0f, 0.01f, new Vec2(-0.0f, -5.0f), 0.0f);

      BodyDef bd = new BodyDef();
      bd.position.set(0.0f, 0.0f);
      ground = getWorld().createBody(bd);
      ground.createFixture(sd, 0f);

      sd.setAsBox(50.0f, 0.01f, new Vec2(0.0f, 25.0f), 0.0f);
      ground.createFixture(sd, 0f);
      sd.setAsBox(0.01f, 50.0f, new Vec2(-15.0f, 0.0f), 0.0f);
      ground.createFixture(sd, 0f);
      sd.setAsBox(0.01f, 50.0f, new Vec2(15.0f, 0.0f), 0.0f);
      ground.createFixture(sd, 0f);
    }

    ConstantVolumeJointDef cvjd = new ConstantVolumeJointDef();

    

    float cx = 0.0f;
    float cy = 10.0f;
    float rx = 5.0f;
    float ry = 5.0f;
    //ArrayList<Body> nbd = new ArrayList<Body>();
    int nBodies = 5;
    float bodyRadius = 0.5f;
    /*
    for (int i = 0; i < nBodies; ++i) {
      float angle = MathUtils.map(i, 0, nBodies, 0, 2 * 3.1415f);
      BodyDef bd = new BodyDef();
      bd.gravityScale = 0;
      bd.linearDamping = .8f;
      // bd.isBullet = true;
      bd.fixedRotation = true;
      
      

      float x = cx + rx * (float) Math.sin(angle);
      float y = cy + ry * (float) Math.cos(angle);
      bd.position.set(new Vec2(x, y));
      bd.type = BodyType.DYNAMIC;
      
      Body body = getWorld().createBody(bd);
      
      //getWorld().queryAABB(callback, aabb);

      FixtureDef fd = new FixtureDef();
      CircleShape cd = new CircleShape();
      cd.m_radius = bodyRadius;
      fd.shape = cd;
      fd.density = 1.0f;
      body.createFixture(fd);
      cvjd.addBody(body);
      g_vertecies.add(body); //***
    } 
    */

    DistanceJointDef j = new DistanceJointDef();
    //j.type = 
    //RopeJointDef j = new RopeJointDef();
    j.length = 5.0f;
    //j.maxLength = 8.0f;
    
    /*
    j.frequencyHz = 10.0f;
    j.dampingRatio = 1.0f;
   
    j.bodyA = g_vertecies.get(0);
    j.bodyB = g_vertecies.get(1);
    getWorld().createJoint(j);
    
    j.bodyA = g_vertecies.get(2);
    j.bodyB = g_vertecies.get(1);
    getWorld().createJoint(j);
    
    j.bodyA = g_vertecies.get(2);
    j.bodyB = g_vertecies.get(3);
    getWorld().createJoint(j);
    
    j.length = 15.0f;
    
    j.bodyA = g_vertecies.get(3);
    j.bodyB = g_vertecies.get(0);
    getWorld().createJoint(j);
    
    j.length = 5.0f;
    j.bodyA = g_vertecies.get(3);
    j.bodyB = g_vertecies.get(1);
    getWorld().createJoint(j);
    
   */
    
    cvjd.frequencyHz = 10.0f;
    cvjd.dampingRatio = 1.0f;
    cvjd.collideConnected = false;
    //getWorld().createJoint(cvjd);
   

    BodyDef bd2 = new BodyDef();
    bd2.type = BodyType.DYNAMIC;
    PolygonShape psd = new PolygonShape();
    psd.setAsBox(3.0f, 1.5f, new Vec2(cx, cy + 15.0f), 0.0f);
    bd2.position = new Vec2(cx, cy + 15.0f);
    //Body fallingBox = getWorld().createBody(bd2);
    //fallingBox.createFixture(psd, 1.0f);
  }
  
  
  /*
  public void mouseDown(Vec2 p, int button, InputEvent rawEvent) {
	  log.info("MouseDown !!!!!");
		log.info(p.toString());
		super.mouseDown(p, button);
		p.x += .3f;
		p.y += .3f;
		
		//cheking mouse collision with vertex
		AABB b = new AABB();
		b.lowerBound.set(p.x - .1f, p.y - .1f);
		b.upperBound.set(p.x + .1f, p.y + .1f);
		log.info("**" +AABB.testOverlap(b, g_vertecies.get(0).getFixtureList().getAABB(0)));
		
		if (rawEvent.isControlDown())
			log.info("Control key was pressed!!!!!!");
		
	}
	*/
  
  public void mouseUp(Vec2 p, int button, InputEvent rawInput) {
	  log.info("MouseUp !!!!!");
	  super.mouseUp(p, button);    
  }
  

  public void mouseDrag(Vec2 p, int button, InputEvent rawInput) {
	  if (this.m_isCreatingEdge){
		  this.mTo = p;
	  } else {
		  this.mouseDrag(p, button);
	  }
  }
  public Body findBody(Vec2 p) {

	    queryAABB.lowerBound.set(p.x - .001f, p.y - .001f);
	    queryAABB.upperBound.set(p.x + .001f, p.y + .001f);
	    callback.point.set(p);
	    callback.fixture = null;
	    m_world.queryAABB(callback, queryAABB);
	    
	    if (callback.fixture != null) {
	      Body body = callback.fixture.getBody();
	      
	      MouseJointDef def = new MouseJointDef();
	      def.bodyA = groundBody;
	      def.bodyB = body;
	      def.collideConnected = true;
	      def.target.set(p);
	      def.maxForce = 1000f * body.getMass();
	      
	      //mouseJoint = (MouseJoint) m_world.createJoint(def);
	      body.setAwake(true);
	      return body;
	    }
	    return null;
	  }
  
  
  public void mouseDown(Vec2 p, int button, InputEvent rawInput) {
		log.debug("mouseDown!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
		if(this.findBody(p) != null)
		{
			
			selectedBody = this.findBody(p); //sets selected body as the one clicked on
			if(recentBody == null) //if null (clicked empty space) sets recent as selected
			{
				recentBody = selectedBody; 
			}
			
			if(g_vertecies.indexOf(recentBody) != g_vertecies.indexOf(selectedBody)) //to make sure not to add an edge to itself
			{
				if (rawInput.isControlDown())
				{
					this.createEdge(g_vertecies.get(g_vertecies.indexOf(recentBody)), 
							g_vertecies.get(g_vertecies.indexOf(selectedBody)));
				} 
			}
			else
			{
				super.mouseDown(p, button);
			}
			recentBody = selectedBody;   //sets recent body as selected for next click
		 }
		else {
			log.debug("Clicked on Empty space!!!!");
			if (rawInput.isControlDown()){
				recentBody = this.createNewVertex(p);  // sets recent body as the one being vreated
				boolean res = g_vertecies.add(recentBody);
				if (res){
					log.debug("there are" + g_vertecies.size() + " vertices in the list");
					
				} else {
					log.debug("OH FAILED!!!!!");
				
				}
			}
			else
			{
				recentBody = null;  //if clicking empty space, set recent body to null 
			}
		}

	}
  

	
public void createEdge(Body v1, Body v2) {
		
		DistanceJointDef j = new DistanceJointDef();
		//calculate distance between the two bodies. 
		float p1 = (float) Math.pow((v2.getPosition().x - v1.getPosition().x), 2); 
		float p2 = (float) Math.pow((v2.getPosition().y - v1.getPosition().y), 2);
		float distance = (float) Math.sqrt(p1 + p2); 
		j.length = distance;    
	    j.frequencyHz = 10.0f;
	    j.dampingRatio = 1.0f;	   
	    j.bodyA = v1;
	    j.bodyB = v2;
	    getWorld().createJoint(j); 
	}
  
  public Body createNewVertex(Vec2 p) {
	  	Body theBall;
	  	
		CircleShape circle = new CircleShape();
		circle.m_radius = 0.5f;
	
		FixtureDef fd = new FixtureDef();
		fd.shape = circle;
		fd.density = 1.0f;
		fd.friction = 0.9f;
	
		BodyDef ballBodyDef = new BodyDef();
		ballBodyDef.gravityScale = 0.0f;
		ballBodyDef.type = BodyType.DYNAMIC;
		ballBodyDef.position.set(p.x, p.y);
		ballBodyDef.linearDamping = 5.0f;
		theBall = m_world.createBody(ballBodyDef);

		theBall.createFixture(fd);
		
		return theBall;
  }

  public String getCustomPanel(){
	  return "org.jbox2d.testbed.framework.j2d.TestbedSidePane.VertexTestPanel";
  }

  @Override
  public String getTestName() {
    return "VERTEX TEST";
  }

}

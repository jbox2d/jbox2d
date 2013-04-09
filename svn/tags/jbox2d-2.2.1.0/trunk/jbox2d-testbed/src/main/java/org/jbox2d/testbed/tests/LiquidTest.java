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
package org.jbox2d.testbed.tests;

import java.util.ArrayList;
import java.util.Arrays;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.MassData;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

// TODO make this liquid usable for developers
/**
 * The dynamic tree broadphase doesn't really suite this test
 * well.
 */
public class LiquidTest extends TestbedTest {
	private boolean firstTime = true;
	
	private int nParticles = 1000;
	private float totalMass = 10.0f;
	private float boxWidth = 2.0f;
	private float boxHeight = 20.0f;
	
	private float fluidMinX = -11.0f;
	private float fluidMaxX = 5.0f;
	private float fluidMinY = -10.0f;
	private float fluidMaxY = 10.0f;
	
	private Body[] liquid;
	
	private float rad = 0.6f;
	private float visc = 0.004f;//0.005f;
	
	private ArrayList<Integer>[][] hash;
	private int hashWidth,hashHeight;
	
	private int hashX(float x) {
		float f = MathUtils.map(x, fluidMinX, fluidMaxX, 0, hashWidth-.001f);
		return (int)f;
	}
	
	private int hashY(float y) {
		float f = MathUtils.map(y,fluidMinY,fluidMaxY,0,hashHeight-.001f);
		return (int)f;
	}
	
	@SuppressWarnings("unchecked")
	public LiquidTest() {
        hash = new ArrayList[40][40];
        for (int i=0; i<40; ++i) {
        	for (int j=0; j<40; ++j) {
        		hash[i][j] = new ArrayList<Integer>();
        	}
        }
        hashWidth = 40;
        hashHeight = 40;
    }
	
	private void hashLocations() {
		for(int a = 0; a < hashWidth; a++)
        {
            for(int b = 0; b < hashHeight; b++){
            	hash[a][b].clear();
            }
        }

        for(int a = 0; a < liquid.length; a++)
        {
            int hcell = hashX(liquid[a].m_sweep.c.x);
            int vcell = hashY(liquid[a].m_sweep.c.y);
            if(hcell > -1 && hcell < hashWidth && vcell > -1 && vcell < hashHeight)
                hash[hcell][vcell].add(new Integer(a));
        }
	}
	
	private void applyLiquidConstraint(float deltaT) {
		/*
		 * Unfortunately, this simulation method is not actually scale
		 * invariant, and it breaks down for rad < ~3 or so.  So we need
		 * to scale everything to an ideal rad and then scale it back after.
		 */
		final float idealRad = 50.0f;
		float multiplier = idealRad / rad;
		
		float[] xchange = new float[liquid.length];
		float[] ychange = new float[liquid.length];
		Arrays.fill(xchange,0.0f);
		Arrays.fill(ychange, 0.0f);
		
		float[] xs = new float[liquid.length];
		float[] ys = new float[liquid.length];
		float[] vxs = new float[liquid.length];
		float[] vys = new float[liquid.length];
		for (int i=0; i<liquid.length; ++i) {
			xs[i] = multiplier*liquid[i].m_sweep.c.x;
			ys[i] = multiplier*liquid[i].m_sweep.c.y;
			vxs[i] = multiplier*liquid[i].m_linearVelocity.x;
			vys[i] = multiplier*liquid[i].m_linearVelocity.y;
		}
		
		for(int i = 0; i < liquid.length; i++) {
			// Populate the neighbor list from the 9 proximate cells
			ArrayList<Integer> neighbors = new ArrayList<Integer>();
	        int hcell = hashX(liquid[i].m_sweep.c.x);
	        int vcell = hashY(liquid[i].m_sweep.c.y);
	        for(int nx = -1; nx < 2; nx++) {
	            for(int ny = -1; ny < 2; ny++) {
	                int xc = hcell + nx;
	                int yc = vcell + ny;
	                if(xc > -1 && xc < hashWidth && yc > -1 && yc < hashHeight && hash[xc][yc].size() > 0) {
	                    for(int a = 0; a < hash[xc][yc].size(); a++) {
	                        Integer ne = (Integer)hash[xc][yc].get(a);
	                        if(ne != null && ne.intValue() != i) neighbors.add(ne);
	                    }
	                }
	            }
	        }
	        
	        // Particle pressure calculated by particle proximity
            // Pressures = 0 iff all particles within range are idealRad distance away
            float[] vlen = new float[neighbors.size()];
            float p = 0.0f;
            float pnear = 0.0f;
            for(int a = 0; a < neighbors.size(); a++) {
                Integer n = (Integer)neighbors.get(a);
                int j = n.intValue();
                float vx = xs[j]-xs[i];//liquid[j].m_sweep.c.x - liquid[i].m_sweep.c.x;
                float vy = ys[j]-ys[i];//liquid[j].m_sweep.c.y - liquid[i].m_sweep.c.y;
                
                //early exit check
                if(vx > -idealRad && vx < idealRad && vy > -idealRad && vy < idealRad) {
                    float vlensqr = (vx * vx + vy * vy);
                    //within idealRad check
                    if(vlensqr < idealRad*idealRad) {
                    	vlen[a] = (float)Math.sqrt(vlensqr);
                    	if (vlen[a] < Settings.EPSILON) vlen[a] = idealRad-.01f;
                        float oneminusq = 1.0f-(vlen[a] / idealRad);
                        p = (p + oneminusq*oneminusq);
                        pnear = (pnear + oneminusq*oneminusq*oneminusq);
                    } else {
                    	vlen[a] = Float.MAX_VALUE;
                    }
                }
            }
            
            // Now actually apply the forces
            //System.out.println(p);
            float pressure = (p - 5F) / 2.0F; //normal pressure term
            float presnear = pnear / 2.0F; //near particles term
            float changex = 0.0F;
            float changey = 0.0F;
            for(int a = 0; a < neighbors.size(); a++) {
                Integer n = (Integer)neighbors.get(a);
                int j = n.intValue();
                float vx = xs[j]-xs[i];//liquid[j].m_sweep.c.x - liquid[i].m_sweep.c.x;
                float vy = ys[j]-ys[i];//liquid[j].m_sweep.c.y - liquid[i].m_sweep.c.y;
                if(vx > -idealRad && vx < idealRad && vy > -idealRad && vy < idealRad) {
                    if(vlen[a] < idealRad) {
                        float q = vlen[a] / idealRad;
                        float oneminusq = 1.0f-q;
                        float factor = oneminusq * (pressure + presnear * oneminusq) / (2.0F*vlen[a]);
                        float dx = vx * factor;
                        float dy = vy * factor;
                        float relvx = vxs[j] - vxs[i];
                        float relvy = vys[j] - vys[i];
                        factor = visc * oneminusq * deltaT;
                        dx -= relvx * factor;
                        dy -= relvy * factor;
                        //liquid[j].m_xf.position.x += dx;//*deltaT*deltaT;
                        //liquid[j].m_xf.position.y += dy;//*deltaT*deltaT;
                        //liquid[j].m_linearVelocity.x += dx;///deltaT;//deltaT;
                        //liquid[j].m_linearVelocity.y += dy;///deltaT;//deltaT;
                        xchange[j] += dx;
                        ychange[j] += dy;
                        changex -= dx;
                        changey -= dy;
                    }
                }
            }
	        //liquid[i].m_xf.position.x += changex;//*deltaT*deltaT;
	        //liquid[i].m_xf.position.y += changey;//*deltaT*deltaT;
	        //liquid[i].m_linearVelocity.x += changex;///deltaT;//deltaT;
	        //liquid[i].m_linearVelocity.y += changey;///deltaT;//deltaT;
	        xchange[i] += changex;
	        ychange[i] += changey;
        }
		//multiplier *= deltaT;
		for (int i=0; i<liquid.length; ++i) {
			liquid[i].m_xf.p.x += xchange[i] / multiplier;
			liquid[i].m_xf.p.y += ychange[i] / multiplier;
			liquid[i].m_linearVelocity.x += xchange[i] / (multiplier*deltaT);
			liquid[i].m_linearVelocity.y += ychange[i] / (multiplier*deltaT);
		}
		
	}
	
	@Override
	public void initTest(boolean argDeserialized) {
		if (firstTime) {
			setCamera(new Vec2(0,2), 35f);
			firstTime = false;
		}
		
		//m_getWorld().setGravity(new Vec2(0.0f,0.0f));
		
		Body ground = null;
		{

			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, 0.0f);
			ground = getWorld().createBody(bd);
			PolygonShape shape = new PolygonShape();
			shape.setAsBox(5.0f, 0.5f);
			ground.createFixture(shape, 0);
			
			shape.setAsBox(1.0f, 0.2f,new Vec2(0.0f,4.0f),-0.2f);
			ground.createFixture(shape, 0);
			shape.setAsBox(1.5f, 0.2f,new Vec2(-1.2f,5.2f),-1.5f);
			ground.createFixture(shape, 0);
			shape.setAsBox(0.5f, 50.0f,new Vec2(5.0f,0.0f),0.0f);
			ground.createFixture(shape, 0);
			
			
			
			shape.setAsBox(0.5f,3.0f,new Vec2(-8.0f,0.0f),0.0f);
			ground.createFixture(shape, 0);
			
			shape.setAsBox(2.0f,0.1f,new Vec2(-6.0f,-2.8f),0.1f);
			ground.createFixture(shape, 0);
			
			CircleShape cd = new CircleShape();
			cd.m_radius = 0.5f;
			cd.m_p.set(-0.5f,-4.0f);
			ground.createFixture(cd, 0);
			
		}
		
		liquid = new Body[nParticles];
		float massPerParticle = totalMass / nParticles;
//		PointDef pd = new PointDef();
//		pd.mass = massPerParticle;
//		pd.restitution = 0.0f;
//		pd.filter.groupIndex = -10;
		CircleShape pd = new CircleShape();
		FixtureDef fd = new FixtureDef();
		fd.shape = pd;
		fd.density = 1f;
		fd.filter.groupIndex = -10;
		pd.m_radius = .05f;
		fd.restitution = 0.4f;
		fd.friction = 0.0f;
		float cx = 0.0f;
		float cy = 25.0f;
		for (int i=0; i<nParticles; ++i) {
			BodyDef bd = new BodyDef();
			bd.position = new Vec2( MathUtils.randomFloat(cx-boxWidth*.5f ,cx+boxWidth*.5f),
					MathUtils.randomFloat(cy-boxHeight*.5f,cy+boxHeight*.5f));
			bd.fixedRotation = true;
			bd.type = BodyType.DYNAMIC;
			Body b = getWorld().createBody(bd);
			
			b.createFixture(fd).setUserData(LIQUID_INT);
			
			MassData md = new MassData();
			md.mass = massPerParticle;
			md.I = 1.0f;
			b.setMassData(md);
			b.setSleepingAllowed(false);
			liquid[i] = b;
		}
		
		PolygonShape polyDef = new PolygonShape();
		polyDef.setAsBox(MathUtils.randomFloat(0.3f,0.7f), MathUtils.randomFloat(0.3f,0.7f));
		BodyDef bodyDef = new BodyDef();
		bodyDef.position = new Vec2(0.0f,25.0f);
		bodyDef.type = BodyType.DYNAMIC;
		bod = getWorld().createBody(bodyDef);
		bod.createFixture(polyDef, 1f);
	}
	
	// IF YOU CHANGE THIS change the corresponding value in World
	public static Integer LIQUID_INT = new Integer(1234598372);
	
	private Body bod;
	
	@Override
	public void step(TestbedSettings settings) {
		super.step(settings);
    float hz = settings.getSetting(TestbedSettings.Hz).value;
		float dt = 1.0f/hz;
		
		int n = 1;
		for (int i=0; i<n; ++i) {
			hashLocations();
			applyLiquidConstraint(dt*n);
		}
		dampenLiquid();
		
		checkBounds();
		
		//System.out.println(m_getWorld().getGroundBody().getXForm());
	}
	
	private void checkBounds() {
		for (int i=0; i<liquid.length; ++i) {
			if (liquid[i].getWorldCenter().y < -10.0f) {
				getWorld().destroyBody(liquid[i]);
				float massPerParticle = totalMass / nParticles;
				
				CircleShape pd = new CircleShape();
				FixtureDef fd = new FixtureDef();
				fd.shape = pd;
				fd.density = 1.0f;
				fd.filter.groupIndex = -10;
				pd.m_radius = .05f;
				fd.restitution = 0.4f;
				fd.friction = 0.0f;
				float cx = 0.0f + MathUtils.randomFloat(-0.6f,0.6f);
				float cy = 15.0f + MathUtils.randomFloat(-2.3f,2.0f);
				BodyDef bd = new BodyDef();
				bd.position = new Vec2( cx, cy );
				bd.fixedRotation = true;
				bd.type = BodyType.DYNAMIC;
				Body b = getWorld().createBody(bd);
				b.createFixture(fd).setUserData(LIQUID_INT);
				MassData md = new MassData();
				md.mass = massPerParticle;
				md.I = 1.0f;
				b.setMassData(md);
				b.setSleepingAllowed(false);
				liquid[i] = b;
			}
		}
		
		if (bod.getWorldCenter().y < -15.0f) {
			getWorld().destroyBody(bod);
			PolygonShape polyDef = new PolygonShape();
			polyDef.setAsBox(MathUtils.randomFloat(0.3f,0.7f), MathUtils.randomFloat(0.3f,0.7f));
			BodyDef bodyDef = new BodyDef();
			bodyDef.position = new Vec2(0.0f,25.0f);
			bodyDef.type = BodyType.DYNAMIC;
			bod = getWorld().createBody(bodyDef);
			bod.createFixture(polyDef, 1f);
		}
	}
	
	private void dampenLiquid() {
		for (int i=0; i<liquid.length; ++i) {
			Body b = liquid[i];
			b.setLinearVelocity(b.getLinearVelocity().mul(0.995f));
		}
	}


	@Override
	public String getTestName() {
		return "Liquid Test";
	}
}

package org.jbox2d.testbed.tests;

import java.util.ArrayList;
import java.util.Arrays;

import org.jbox2d.collision.MassData;
import org.jbox2d.collision.shapes.CircleDef;
import org.jbox2d.collision.shapes.PolygonDef;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;

import processing.core.PApplet;

// TODO make this liquid usable for developers
public class LiquidTest extends AbstractExample {
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
		float f = PApplet.map(x, fluidMinX, fluidMaxX, 0, hashWidth-.001f);
		return (int)f;
	}
	
	private int hashY(float y) {
		float f = PApplet.map(y,fluidMinY,fluidMaxY,0,hashHeight-.001f);
		return (int)f;
	}
	
	@SuppressWarnings("unchecked")
	public LiquidTest(TestbedMain t) {
        super(t);
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
			liquid[i].m_xf.position.x += xchange[i] / multiplier;
			liquid[i].m_xf.position.y += ychange[i] / multiplier;
			liquid[i].m_linearVelocity.x += xchange[i] / (multiplier*deltaT);
			liquid[i].m_linearVelocity.y += ychange[i] / (multiplier*deltaT);
		}
		
	}
	
	@Override
	public void create() {
		if (firstTime) {
			setCamera(0f, 2f, 35f);
			firstTime = false;
		}
		
		//m_world.setGravity(new Vec2(0.0f,0.0f));
		
		Body ground = null;
		{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(5.0f, 0.5f);

			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, 0.0f);
			ground = m_world.createBody(bd);
			ground.createShape(sd);
			sd.setAsBox(1.0f, 0.2f,new Vec2(0.0f,4.0f),-0.2f);
			ground.createShape(sd);
			sd.setAsBox(1.5f, 0.2f,new Vec2(-1.2f,5.2f),-1.5f);
			ground.createShape(sd);
			sd.setAsBox(0.5f, 50.0f,new Vec2(5.0f,0.0f),0.0f);
			ground.createShape(sd);
			
			
			
			sd.setAsBox(0.5f,3.0f,new Vec2(-8.0f,0.0f),0.0f);
			ground.createShape(sd);
			
			sd.setAsBox(2.0f,0.1f,new Vec2(-6.0f,-2.8f),0.1f);
			ground.createShape(sd);
			
			CircleDef cd = new CircleDef();
			cd.radius = 0.5f;
			cd.localPosition.set(-0.5f,-4.0f);
			ground.createShape(cd);
			
		}
		
		liquid = new Body[nParticles];
		float massPerParticle = totalMass / nParticles;
//		PointDef pd = new PointDef();
//		pd.mass = massPerParticle;
//		pd.restitution = 0.0f;
//		pd.filter.groupIndex = -10;
		CircleDef pd = new CircleDef();
		pd.density = 1.0f;
		pd.filter.groupIndex = -10;
		pd.radius = .05f;
		pd.restitution = 0.4f;
		pd.friction = 0.0f;
		float cx = 0.0f;
		float cy = 25.0f;
		for (int i=0; i<nParticles; ++i) {
			BodyDef bd = new BodyDef();
			bd.position = new Vec2( parent.random(cx-boxWidth*.5f ,cx+boxWidth*.5f),
									parent.random(cy-boxHeight*.5f,cy+boxHeight*.5f));
			bd.fixedRotation = true;
			Body b = m_world.createBody(bd);
			b.createShape(pd).setUserData(LIQUID_INT);
			MassData md = new MassData();
			md.mass = massPerParticle;
			md.I = 1.0f;
			b.setMass(md);
			b.allowSleeping(false);
			liquid[i] = b;
		}
		
		PolygonDef polyDef = new PolygonDef();
		polyDef.setAsBox(parent.random(0.3f,0.7f), parent.random(0.3f,0.7f));
		polyDef.density = 1.0f;
		BodyDef bodyDef = new BodyDef();
		bodyDef.position = new Vec2(0.0f,25.0f);
		bod = m_world.createBody(bodyDef);
		bod.createShape(polyDef);
		bod.setMassFromShapes();
		
	}
	
	// IF YOU CHANGE THIS change the corresponding value in World
	public static Integer LIQUID_INT = new Integer(12345);
	
	private Body bod;
	
	public void postStep() {
		//if (true) return;
		float dt = 1.0f/this.settings.hz;
		
		int n = 1;
		for (int i=0; i<n; ++i) {
			hashLocations();
			applyLiquidConstraint(dt*n);
		}
		dampenLiquid();
		
		checkBounds();
		
		//System.out.println(m_world.getGroundBody().getXForm());
	}
	
	private void checkBounds() {
		for (int i=0; i<liquid.length; ++i) {
			if (liquid[i].getMemberWorldCenter().y < -10.0f) {
				m_world.destroyBody(liquid[i]);
				float massPerParticle = totalMass / nParticles;
				CircleDef pd = new CircleDef();
				pd.density = 1.0f;
				pd.filter.groupIndex = -10;
				pd.radius = .05f;
				pd.restitution = 0.4f;
				pd.friction = 0.0f;
				float cx = 0.0f + parent.random(-0.6f,0.6f);
				float cy = 15.0f + parent.random(-2.3f,2.0f);
				BodyDef bd = new BodyDef();
				bd.position = new Vec2( cx, cy );
				bd.fixedRotation = true;
				Body b = m_world.createBody(bd);
				b.createShape(pd).setUserData(LIQUID_INT);
				MassData md = new MassData();
				md.mass = massPerParticle;
				md.I = 1.0f;
				b.setMass(md);
				b.allowSleeping(false);
				liquid[i] = b;
			}
		}
		
		if (bod.getMemberWorldCenter().y < -15.0f) {
			m_world.destroyBody(bod);
			PolygonDef polyDef = new PolygonDef();
			polyDef.setAsBox(parent.random(0.3f,0.7f), parent.random(0.3f,0.7f));
			polyDef.density = 1.0f;
			BodyDef bodyDef = new BodyDef();
			bodyDef.position = new Vec2(0.0f,25.0f);
			bod = m_world.createBody(bodyDef);
			bod.createShape(polyDef);
			bod.setMassFromShapes();
		}
	}
	
	private void dampenLiquid() {
		for (int i=0; i<liquid.length; ++i) {
			Body b = liquid[i];
			b.setLinearVelocity(b.getLinearVelocity().mul(0.995f));
		}
	}

	@Override
	public String getName() {
		return "Liquid Test";
	}

}

/**
 * 
 */
package org.jbox2d.dynamics.controllers;

import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.DebugDraw;
import org.jbox2d.dynamics.TimeStep;

/**
 * @author eric
 *
 */
public class BuoyancyController extends Controller {
	/**
	 * @param def
	 */
	protected BuoyancyController(BuoyancyControllerDef def) {
		super(def);
		normal = def.normal.clone();
		offset = def.offset;
		density = def.density;
		velocity = def.velocity.clone();
		linearDrag = def.linearDrag;
		angularDrag = def.angularDrag;
		useDensity = def.useDensity;
		useWorldGravity = def.useWorldGravity;
		gravity = def.gravity.clone();
	}
	
	/** The outer surface normal */
	public Vec2 normal = new Vec2();
	/** The height of the fluid surface along the normal */
	public float offset;
	/** The fluid density */
	public float density;
	/** Fluid velocity, for drag calculations */
	public Vec2 velocity = new Vec2();
	/** Linear drag co-efficient */
	public float linearDrag;
	/** Linear drag co-efficient */
	public float angularDrag;
	/** If false, bodies are assumed to be uniformly dense, otherwise use the shapes densities */
	public boolean useDensity; //False by default to prevent a gotcha
	/** If true, gravity is taken from the world instead of the gravity parameter. */
	public boolean useWorldGravity;
	/** Gravity vector, if the world's gravity is not used */
	public Vec2 gravity = new Vec2();
	
	@Override
	public void step(final TimeStep step) {
		if(m_bodyList == null) return;
		if(useWorldGravity) {
			gravity = m_world.getGravity();
		}
		for(ControllerEdge i=m_bodyList;i!=null;i=i.nextBody) {
			Body body = i.body;
			if(body.isSleeping()) {
				//Buoyancy force is just a function of position,
				//so unlike most forces, it is safe to ignore sleeping bodes
				continue;
			}
			Vec2 areac = new Vec2(0,0);
			Vec2 massc = new Vec2(0,0);
			float area = 0;
			float mass = 0;
			for(Shape shape=body.getShapeList();shape!=null;shape=shape.getNext()) {
				Vec2 sc = new Vec2(0,0);
				float sarea = shape.computeSubmergedArea(normal, offset, sc);
				area += sarea;
				areac.x += sarea * sc.x;
				areac.y += sarea * sc.y;
				float shapeDensity = 0;
				if(useDensity) {
					//TODO: Expose density publicly
					shapeDensity=shape.getDensity();
				}
				else
				{
					shapeDensity = 1;
				}
				mass += sarea*shapeDensity;
				massc.x += sarea * sc.x * shapeDensity;
				massc.y += sarea * sc.y * shapeDensity;
			}
			areac.x/=area;
			areac.y/=area;
			//Vec2 localCentroid = XForm.mulTrans(body.getXForm(),areac);
			massc.x/=mass;
			massc.y/=mass;
			if(area<Settings.EPSILON) continue;
			//Buoyancy
			Vec2 buoyancyForce = gravity.mul(-density*area);
			body.applyForce(buoyancyForce,massc);
			//Linear drag
			Vec2 dragForce = body.getLinearVelocityFromWorldPoint(areac).sub(velocity);
			dragForce.mulLocal(-linearDrag*area);
			body.applyForce(dragForce,areac);
			//Angular drag
			//TODO: Something that makes more physical sense?
			body.applyTorque(-body.getInertia()/body.getMass()*area*body.getAngularVelocity()*angularDrag);
			
		}
	}
	
	@Override
	public void draw(DebugDraw debugDraw) {
		float r = 1000;
		Vec2 p1 = normal.mul(offset).addLocal(Vec2.cross(normal, r));
		Vec2 p2 = normal.mul(offset).subLocal(Vec2.cross(normal, r));

		Color3f color = new Color3f(0,0,255*0.8f);

//		debugDraw.drawSegment(p1, p2, color);
		Vec2[] vertices = new Vec2[] { p1, p2, p2.sub(normal.mul(r)), p1.sub(normal.mul(r))};
		debugDraw.drawSolidPolygon(vertices, 4, color);
	}
}

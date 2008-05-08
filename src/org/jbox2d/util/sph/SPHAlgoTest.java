package org.jbox2d.util.sph;

import org.jbox2d.collision.AABB;
import org.jbox2d.common.Vec2;

import processing.core.PApplet;

public class SPHAlgoTest extends PApplet {
	public SPHGroup sphGroup;
	public float tStep = 1.0f/30.0f;
	public void setup() {
		size(640,480,P3D);
		frameRate(100);
		AABB aabb = new AABB(new Vec2(0f,0f),new Vec2(width,height));
		sphGroup = new SPHGroup(100,aabb,this, tStep);
	}
	int count = 0;
	public void draw() {
		System.out.println(count++);
		background(0);
		float GRAVITY = 9.8f;
		sphGroup.updateParticles();
		stroke(255);
		for (int i=0; i<sphGroup.t.length; ++i) {
			sphGroup.t[i].vel.y += GRAVITY*tStep;
			if (sphGroup.t[i].pos.y > height) {
				sphGroup.t[i].vel.y *= -0.95f;
				sphGroup.t[i].pos.y = height-.1f;
			}
			ellipse(sphGroup.t[i].pos.x,sphGroup.t[i].pos.y,2.0f,2.0f);
		}
	}
	
	
	/** Start PApplet as a Java program (can also be run as an applet). */
    static public void main(String args[]) {
        PApplet.main(new String[] { "org.jbox2d.util.sph.SPHAlgoTest" });
    }
}

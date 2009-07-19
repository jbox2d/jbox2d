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

package org.jbox2d.testbed;


import org.jbox2d.common.Color3f;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.ViewportTransform;
import org.jbox2d.common.XForm;
import org.jbox2d.dynamics.DebugDraw;

import processing.core.PApplet;
import processing.core.PFont;
import processing.core.PGraphics3D;
import processing.core.PImage;

/**
 * Implementation of DebugDraw using Processing (http://www.processing.org)
 * 
 * @author ewjordan
 */
public class ProcessingDebugDraw extends DebugDraw {
	public static ProcessingDebugDraw screen; //static rendering context for debug drawing from within other code

	private boolean firstTime = true;
	public PApplet g;
	public PFont m_font;
	public float fontHeight;
	// World 0,0 maps to transX, transY on screen
	
	public ProcessingDebugDraw(PApplet pApplet) {
		super(new ViewportTransform());
		g = pApplet;
		screen = this;
		m_font = g.createFont("LucidaGrande-Bold",12);//-Bold-14.vlw");
		fontHeight = 14.0f;
		viewportTransform.setTransform( Mat22.createScaleTransform( 20));
		viewportTransform.setCenter(320 + g.width/2, 240 + g.height/2);
		viewportTransform.setExtents( g.width/2, g.height/2);
		viewportTransform.yFlip = true;
	}

	private static final Vec2 circlePt = new Vec2();
	/* (non-Javadoc)
	 * @see org.jbox2d.dynamics.DebugDraw#drawCircle(org.jbox2d.common.Vec2, float, javax.vecmath.Color3f)
	 */
	@Override
	public void drawCircle(Vec2 argCenter, float radius, Color3f color) {
		viewportTransform.getWorldToScreenToOut(argCenter, center);

		g.noFill();
		float k_segments = 16.0f;
		float k_increment = 2.0f * (float)Math.PI / k_segments;
		float theta = 0.0f;
		g.stroke(color.x, color.y, color.z);
		g.noFill();
		g.beginShape(PApplet.POLYGON);
		for (int i = 0; i < k_segments; ++i) {
			float vx = center.x + radius * (float)Math.cos(theta);
			float vy = center.y + radius * (float)Math.sin(theta);
			circlePt.set( vx, vy);
			Mat22.mulToOut( viewportTransform.getTransform(), circlePt, circlePt);
			g.vertex(circlePt.x, circlePt.y);
			theta += k_increment;
		}
		g.vertex(center.x + radius, center.y);
		g.endShape();
	}

	
	// djm pooling
	private static final Vec2 p = new Vec2();
	private static final Vec2 center = new Vec2();
	/* (non-Javadoc)
	 * @see org.jbox2d.dynamics.DebugDraw#drawSolidCircle(org.jbox2d.common.Vec2, float, org.jbox2d.common.Vec2, javax.vecmath.Color3f)
	 */
	@Override
	public void drawSolidCircle(Vec2 argCenter, float radius, Vec2 axis, Color3f color) {
		viewportTransform.getWorldToScreenToOut(argCenter, center);
		
		float k_segments = 16.0f;
		float k_increment = 2.0f * (float)Math.PI / k_segments;
		float theta = 0.0f;
		g.fill(0.5f*color.x, 0.5f*color.y, 0.5f*color.z, 0.5f*255.0f);
		g.stroke(color.x,color.y,color.z, 255.0f);
		g.beginShape(PApplet.POLYGON);
		for (int i = 0; i < k_segments; ++i) {
			float vx = radius * (float)Math.cos(theta);
			float vy = radius * (float)Math.sin(theta);
			circlePt.set( vx, vy);
			Mat22.mulToOut( viewportTransform.getTransform(), circlePt, circlePt);
			circlePt.addLocal( center);
			g.vertex(circlePt.x, circlePt.y);
			theta += k_increment;
		}
		circlePt.set( radius, 0);
		Mat22.mulToOut( viewportTransform.getTransform(), circlePt, circlePt);
		circlePt.addLocal( center);
		g.vertex(circlePt.x, circlePt.y);
		
		g.endShape();

		p.set(center.x + radius * axis.x, center.y + radius * axis.y);
		g.beginShape(PApplet.LINES);
		g.vertex(center.x, center.y);
		g.vertex(p.x, p.y);
		g.endShape();
	}

	// djm pooling
	private static final Vec2 v1 = new Vec2();
	private static final Vec2 v2 = new Vec2();
	@Override
	public void drawPolygon(Vec2[] vertices, int vertexCount, Color3f color) {
		g.stroke(color.x, color.y, color.z);
		g.noFill();
		for (int i = 0; i < vertexCount; ++i) {
			int ind = (i+1<vertexCount)?i+1:(i+1-vertexCount);
			viewportTransform.getWorldToScreenToOut(vertices[i], v1);
			viewportTransform.getWorldToScreenToOut(vertices[ind], v2);
			//System.out.println(v1 + " -> "+v2);
			g.line(v1.x, v1.y, v2.x, v2.y);
		}
	}
	
	// djm pooling, and from above
	private static final Vec2 v = new Vec2();
	@Override
	public void drawSolidPolygon(Vec2[] vertices, int vertexCount, Color3f color) {
		g.noStroke();
		g.fill(0.5f * color.x, 0.5f * color.y, 0.5f * color.z, 0.5f*255.0f);
		g.beginShape(PApplet.POLYGON);
		for (int i = 0; i < vertexCount; ++i) {
			viewportTransform.getWorldToScreenToOut(vertices[i], v);
			g.vertex(v.x, v.y);
		}
		g.endShape();
		
		g.stroke(color.x, color.y, color.z, 255.0f);
		for (int i = 0; i < vertexCount; ++i) {
			int ind = (i+1<vertexCount)?i+1:(i+1-vertexCount);
			viewportTransform.getWorldToScreenToOut(vertices[i], v1);
			viewportTransform.getWorldToScreenToOut(vertices[ind], v2);
			//System.out.println("Drawing: "+v1+" to "+v2);
			g.line(v1.x, v1.y, v2.x, v2.y);
		}
	}

	@Override
	public void drawSegment(Vec2 argP1, Vec2 argP2, Color3f color) {
		viewportTransform.getWorldToScreenToOut(argP1, p1);
		viewportTransform.getWorldToScreenToOut(argP2, p2);
		g.stroke(color.x, color.y, color.z);
		g.beginShape(PApplet.LINES);
		g.vertex(p1.x, p1.y);
		g.vertex(p2.x, p2.y);
		g.endShape();
	}
	
	// djm pooling
	private static final Vec2 p1 = new Vec2();
	private static final Vec2 p2 = new Vec2();
	private static final Vec2 p1world = new Vec2();
	private static final Vec2 p2world = new Vec2();
	/* (non-Javadoc)
	 * @see org.jbox2d.dynamics.DebugDraw#drawXForm(org.jbox2d.common.XForm)
	 */
	@Override
	public void drawXForm(XForm xf) {
		p1.set(xf.position);
		p2.setZero();
		float k_axisScale = 0.4f;
		g.beginShape(PApplet.LINES);
		viewportTransform.getWorldToScreenToOut(p1, p1world);
		g.stroke(255.0f, 0.0f, 0.0f);
		g.vertex(p1world.x, p1world.y);
		p2.x = p1.x + k_axisScale * xf.R.col1.x;
		p2.y = p1.y + k_axisScale * xf.R.col1.y;
		viewportTransform.getWorldToScreenToOut(p2, p2world);
		g.vertex(p2world.x, p2world.y);

		g.stroke(0.0f, 255.0f, 0.0f);
		g.vertex(p1world.x, p1world.y);
		p2.x = p1.x + k_axisScale * xf.R.col2.x;
		p2.y = p1.y + k_axisScale * xf.R.col2.y;
		viewportTransform.getWorldToScreenToOut(p2, p2world);
		g.vertex(p2world.x, p2world.y);

		g.endShape();
	}
	
	@Override
	public void drawString(float x, float y, String s, Color3f color) {
		//g.textFont(m_font, 36);
		//if (true) return;
		if (firstTime) {
			g.textFont(m_font);
			if (g.g instanceof PGraphics3D) g.textMode(PApplet.SCREEN);
			firstTime = false;
		}
		g.fill(color.x,color.y,color.z);
		//g.fill(255.0f);
		g.text(s, x, y);
	}

	private static final Vec2 position = new Vec2();
	@Override
	public void drawPoint(Vec2 argPosition, float f, Color3f color) {
		viewportTransform.getWorldToScreenToOut(argPosition, position);
		float k_segments = 5.0f;
		float k_increment = 2.0f * (float)Math.PI / k_segments;
		float k_radius = 3.0f;
		float theta = 0.0f;
		g.fill(color.x, color.y, color.z);
		g.noStroke();
		g.beginShape(PApplet.POLYGON);
		for (int i = 0; i < k_segments; ++i) {
			float vx = position.x + k_radius * (float)Math.cos(theta);
			float vy = position.y + k_radius * (float)Math.sin(theta);
			g.vertex(vx, vy);
			theta += k_increment;
		}
		g.endShape();
	}
	
	private static final Vec2 localOffset = new Vec2();
	/**
	 * First image is centered on position, then
	 * rotation, then localOffset is applied, and
     * lastly localScale.
     * <BR><BR>
     * Thus localOffset should be specified in world
     * units before scaling is applied.
     * For instance, if you want a MxN image to have its corner
     * at body center and be scaled by S, use a localOffset
     * of (M*S/2, N*S/2) and a localScale of S.
     * <BR><BR>
     * 
     */
	public void drawImage(PImage image, Vec2 argPosition, float rotation, float localScale,
						  Vec2 argLocalOffset, float halfImageWidth, float halfImageHeight) {
		viewportTransform.getWorldToScreenToOut(argPosition, position);
		viewportTransform.getTransform().mulToOut(argLocalOffset, localOffset);
        g.pushMatrix();
        g.translate(position.x, position.y);
        g.rotate(-rotation);
        g.translate(localOffset.x, localOffset.y);
        g.scale( localScale);
        Mat22 mat = viewportTransform.getTransform();
        g.applyMatrix( mat.col1.x, mat.col2.x, 0, 0,
                       mat.col1.y, mat.col2.y, 0, 0,
                       0, 0, 1, 0,
                       0, 0, 0, 1);
        g.image(image, -halfImageWidth, -halfImageHeight);
        g.popMatrix();
    }
}

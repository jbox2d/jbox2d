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
import org.jbox2d.common.Vec2;
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
    public float transX = 320.0f;
    public float transY = 240.0f;
    public float scaleFactor = 20.0f;
    public float yFlip = -1.0f; //flip y coordinate
    
    public void setCamera(float x, float y, float scale) {
    	transX = PApplet.map(x,0.0f,-1.0f,g.width*.5f,g.width*.5f+scale);
    	transY = PApplet.map(y,0.0f,yFlip*1.0f,g.height*.5f,g.height*.5f+scale);
    	scaleFactor = scale;
    }
	
	public ProcessingDebugDraw(PApplet pApplet) {
		screen = this;
		g = pApplet;
		m_font = g.createFont("LucidaGrande-Bold",12);//-Bold-14.vlw");
		fontHeight = 14.0f;
	}
	
	public Vec2 worldToScreen(Vec2 world) {
		float x = PApplet.map(world.x, 0f, 1f, transX, transX+scaleFactor);
		float y = PApplet.map(world.y, 0f, 1f, transY, transY+scaleFactor);
		if (yFlip == -1.0f) y = PApplet.map(y,0f,g.height, g.height,0f);
		return new Vec2(x, y);
	}
	public Vec2 worldToScreen(float x, float y) {
		return worldToScreen(new Vec2(x,y));
	}
	
	public Vec2 screenToWorld(Vec2 screen) {
		float x = PApplet.map(screen.x, transX, transX+scaleFactor, 0f, 1f);
		float y = screen.y;
		if (yFlip == -1.0f) y = PApplet.map(y,g.height,0f,0f,g.height);
		y = PApplet.map(y, transY, transY+scaleFactor, 0f, 1f);
		return new Vec2(x,y);
	}
	public Vec2 screenToWorld(float x, float y) {
		return screenToWorld(new Vec2(x,y));
	}

	/* (non-Javadoc)
	 * @see org.jbox2d.dynamics.DebugDraw#drawCircle(org.jbox2d.common.Vec2, float, javax.vecmath.Color3f)
	 */
	@Override
	public void drawCircle(Vec2 center, float radius, Color3f color) {
		center = worldToScreen(center);
		radius *= scaleFactor;
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
			g.vertex(vx, vy);
			theta += k_increment;
		}
		g.vertex(center.x + radius, center.y);
		g.endShape();
	}

	
	/* (non-Javadoc)
	 * @see org.jbox2d.dynamics.DebugDraw#drawSolidCircle(org.jbox2d.common.Vec2, float, org.jbox2d.common.Vec2, javax.vecmath.Color3f)
	 */
	@Override
	public void drawSolidCircle(Vec2 center, float radius, Vec2 axis,
			Color3f color) {
		center = worldToScreen(center);
		radius = radius * scaleFactor;
		axis = new Vec2(axis.x, axis.y*yFlip);
		
		float k_segments = 16.0f;
		float k_increment = 2.0f * (float)Math.PI / k_segments;
		float theta = 0.0f;
		g.fill(0.5f*color.x, 0.5f*color.y, 0.5f*color.z, 0.5f*255.0f);
		g.stroke(color.x,color.y,color.z, 255.0f);
		g.beginShape(PApplet.POLYGON);
		for (int i = 0; i < k_segments; ++i) {
			float vx = center.x + radius * (float)Math.cos(theta);
			float vy = center.y + radius * (float)Math.sin(theta);
			g.vertex(vx, vy);
			theta += k_increment;
		}
		g.vertex(center.x+radius, center.y);
		g.endShape();

		Vec2 p = new Vec2(center.x + radius * axis.x, center.y + radius * axis.y);
		g.beginShape(PApplet.LINES);
		g.vertex(center.x, center.y);
		g.vertex(p.x, p.y);
		g.endShape();
	}

	/* (non-Javadoc)
	 * @see org.jbox2d.dynamics.DebugDraw#drawPolygon(org.jbox2d.common.Vec2[], int, javax.vecmath.Color3f)
	 */
	@Override
	public void drawPolygon(Vec2[] vertices, int vertexCount, Color3f color) {
		g.stroke(color.x, color.y, color.z);
		g.noFill();
		for (int i = 0; i < vertexCount; ++i) {
			int ind = (i+1<vertexCount)?i+1:(i+1-vertexCount);
			Vec2 v1 = worldToScreen(vertices[i]);
			Vec2 v2 = worldToScreen(vertices[ind]);
			g.line(v1.x, v1.y, v2.x, v2.y);
		}

	}
	
	/* (non-Javadoc)
	 * @see org.jbox2d.dynamics.DebugDraw#drawSolidPolygon(org.jbox2d.common.Vec2[], int, javax.vecmath.Color3f)
	 */
	@Override
	public void drawSolidPolygon(Vec2[] vertices, int vertexCount, Color3f color) {
		g.noStroke();
		g.fill(0.5f * color.x, 0.5f * color.y, 0.5f * color.z, 0.5f*255.0f);
		g.beginShape(PApplet.POLYGON);
		for (int i = 0; i < vertexCount; ++i) {
			Vec2 v = worldToScreen(vertices[i]);
			g.vertex(v.x, v.y);
		}
		g.endShape();
		
		g.stroke(color.x, color.y, color.z, 255.0f);
		for (int i = 0; i < vertexCount; ++i) {
			int ind = (i+1<vertexCount)?i+1:(i+1-vertexCount);
			Vec2 v1 = worldToScreen(vertices[i]);
			Vec2 v2 = worldToScreen(vertices[ind]);
			g.line(v1.x, v1.y, v2.x, v2.y);
		}
	}

	@Override
	public void drawSegment(Vec2 p1, Vec2 p2, Color3f color) {
		p1 = worldToScreen(p1);
		p2 = worldToScreen(p2);
		g.stroke(color.x, color.y, color.z);
		g.beginShape(PApplet.LINES);
		g.vertex(p1.x, p1.y);
		g.vertex(p2.x, p2.y);
		g.endShape();
	}
		
	/* (non-Javadoc)
	 * @see org.jbox2d.dynamics.DebugDraw#drawXForm(org.jbox2d.common.XForm)
	 */
	@Override
	public void drawXForm(XForm xf) {
		Vec2 p1 = xf.position.clone(), p2 = new Vec2();
		float k_axisScale = 0.4f;
		g.beginShape(PApplet.LINES);
		Vec2 p1world = worldToScreen(p1);
		g.stroke(1.0f, 0.0f, 0.0f);
		g.vertex(p1world.x, p1world.y);
		p2.x = p1.x + k_axisScale * xf.R.col1.x;
		p2.y = p1.y + k_axisScale * xf.R.col1.y;
		Vec2 p2world = worldToScreen(p2);
		g.vertex(p2world.x, p2world.y);

		g.stroke(0.0f, 1.0f, 0.0f);
		g.vertex(p1world.x, p1world.y);
		p2.x = p1.x + k_axisScale * xf.R.col2.x;
		p2.y = p1.x + k_axisScale * xf.R.col2.y;
		p2world = worldToScreen(p2);
		g.vertex(p2.x, p2.y);

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

	@Override
	public void drawPoint(Vec2 position, float f, Color3f color) {
		position = worldToScreen(position);
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
	
	/**
	 * First image is centered on position, then
	 * localScale is applied, then localOffset, and
     * lastly rotation.
     * <BR><BR>
     * Thus localOffset should be specified in world
     * units before scaling is applied.
     * For instance, if you want a MxN image to have its corner
     * at body center and be scaled by S, use a localOffset
     * of (M*S/2, N*S/2) and a localScale of S.
     * <BR><BR>
     * 
     */
	public void drawImage(PImage image, Vec2 position, float rotation, float localScale,
						  Vec2 localOffset, float halfImageWidth, float halfImageHeight) {
		position = worldToScreen(position);
		localOffset = worldToScreenVector(localOffset);
		localScale *= scaleFactor;
        g.pushMatrix();
        g.translate(position.x, position.y);
        g.rotate(-rotation);
        g.translate(localOffset.x, localOffset.y);
        g.scale(localScale);
        g.image(image, -halfImageWidth, -halfImageHeight);
        g.popMatrix();
    }
	
	public Vec2 worldToScreenVector(Vec2 world) {
		return world.mul(scaleFactor);
	}
	public Vec2 worldToScreenVector(float x, float y) {
		return worldToScreenVector(new Vec2(x,y));
	}
	
}

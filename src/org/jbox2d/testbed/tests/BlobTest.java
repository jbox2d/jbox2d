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

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.PolygonDef;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;
import org.jbox2d.util.blob.*;

public class BlobTest extends AbstractExample {
	private boolean firstTime;
	
	public BlobTest(TestbedMain _parent) {
		super(_parent);
		firstTime = true;
	}
	
	@Override
	public void create() {

		if (firstTime) {
			setCamera(0.0f,10.0f,20.0f);
			firstTime = false;
		}
		
    	Body ground = null;
		{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(50.0f, 0.2f);

			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, 0.0f);
			ground = m_world.createBody(bd);
			ground.createShape(sd);
		}
		
		BlobStructure structure = new SquareLatticeStructure();
		structure.setSpringDamping(0.1f);
		structure.setSpringFrequency(3f);
		BlobContainer container = new CircularBlobContainer(new Vec2(0.0f,12.0f),6.0f);
		/*
		BlobMaker.pointRadius = 0.6f;
		BlobMaker.pointFriction = 0.2f;
		BlobMaker.pointDensity = 0.1f;
		BlobMaker.createBlob(structure, container, m_world,
							//scaleX   scaleY    shiftX  shiftY
							1.5f,     1.25f,    0.0f,  0.15f);
		*/
		
		container = new DonutBlobContainer(new Vec2(0.0f,28.0f),1.0f,3.0f);
		//container = new CircularBlobContainer(new Vec2(0.0f,28.0f),3.0f);
		/*container = new BlobContainer(){
			AABB aabb = new AABB(new Vec2(-11f,10f), new Vec2(-8f,15f));
			public boolean containsPoint(Vec2 p) { 
				if (p.x < aabb.lowerBound.x || p.x > aabb.upperBound.x ||
					p.y < aabb.lowerBound.y || p.y > aabb.upperBound.y)
					return false;
				return true;
			}

			public AABB getAABB() {
				return aabb;
			}
		};*/
		structure.setSpringFrequency(30.0f);
		BlobMaker.pointRadius = 0.2f;
		BlobMaker.createBlob(structure, container, m_world,
				//scaleX   scaleY    shiftX  shiftY
				0.3f,      0.4f,     0.2f,  0.3f);
		

	}

	@Override
	public String getName() {
		return "BlobMaker Test";
	}

}
/**
 * Created at 8:41:50 PM Jan 23, 2011
 */
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class DominoTower extends TestbedTest {
	private boolean firstTime = true;
    final float dwidth = .20f;
    final float dheight = 1.0f;
    float ddensity;// = 10f;
    final float dfriction = 0.1f;
    int baseCount = 25;
    
    public void makeDomino(float x, float y, boolean horizontal, World world) {

    	PolygonShape sd = new PolygonShape();
        sd.setAsBox(.5f*dwidth, .5f*dheight);
        FixtureDef fd = new FixtureDef();
        fd.shape = sd;
        fd.density = ddensity;
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        fd.friction = dfriction;
        fd.restitution = 0.65f;
        bd.position = new Vec2(x, y);
        bd.angle = horizontal? (float)(Math.PI/2.0):0f;
        Body myBody = world.createBody(bd);
        myBody.createFixture(fd);
    }
    
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#initTest()
	 */
	@Override
	public void initTest() {
		if (firstTime) {
			setCamera(0f, 12f, 10f);
			firstTime = false;
		}
    	
        { // Floor
            PolygonShape sd = new PolygonShape();
            sd.setAsBox(50.0f, 10.0f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f, -10.0f);
            m_world.createBody(bd).createFixture(sd, 0f);
        }
        
        {
            ddensity = 10f;
            //Make bullet
            PolygonShape sd = new PolygonShape();
            sd.setAsBox(.7f, .7f);
            FixtureDef fd = new FixtureDef();
            fd.density = 35f;
            BodyDef bd = new BodyDef();
            bd.type = BodyType.DYNAMIC;
            fd.shape = sd;
            fd.friction = 0f;
            fd.restitution = 0.85f;
            bd.bullet = true;
            //bd.addShape(sd);
            bd.position = new Vec2(30f, 50f);
            Body b = m_world.createBody(bd);
            b.createFixture(fd);
            b.setLinearVelocity(new Vec2(-25f,-25f));
            b.setAngularVelocity(6.7f);
            
            fd.density = 25f;
            bd.position = new Vec2(-30, 25f);
            b = m_world.createBody(bd);
            b.createFixture(fd);
            b.setLinearVelocity(new Vec2(35f, -10f));
            b.setAngularVelocity(-8.3f);
        }

        { 
            float currX;
            //Make base
            for (int i=0; i<baseCount; ++i) {
                currX = i*1.5f*dheight - (1.5f*dheight*baseCount/2f);
                makeDomino(currX, dheight/2.0f, false, m_world);
                makeDomino(currX, dheight+dwidth/2.0f, true, m_world);
            }
            currX = baseCount*1.5f*dheight - (1.5f*dheight*baseCount/2f);            
            //Make 'I's
            for (int j=1; j<baseCount; ++j) {
                if (j > 3) ddensity *= .8f;
                float currY = dheight*.5f + (dheight+2f*dwidth)*.99f*j; //y at center of 'I' structure
                
                for (int i=0; i<baseCount - j; ++i) {
                    currX = i*1.5f*dheight - (1.5f*dheight*(baseCount-j)/2f);// + parent.random(-.05f, .05f);
                    ddensity *= 2.5f;
                    if (i==0) {
                        makeDomino(currX - (1.25f*dheight) + .5f*dwidth, currY-dwidth, false, m_world);
                    }
                    if (i==baseCount-j-1) {
                        /*if (j != 1) djm: why is this here? it makes it off balance*/
                    	makeDomino(currX + (1.25f*dheight) - .5f*dwidth, currY-dwidth, false, m_world);
                    }
                    ddensity /= 2.5f;
                    makeDomino(currX, currY, false, m_world);
                    makeDomino(currX, currY+.5f*(dwidth+dheight), true, m_world);
                    makeDomino(currX, currY-.5f*(dwidth+dheight), true, m_world);
                }
            }
        }
	}
	
	/**
	 * @see org.jbox2d.testbed.framework.TestbedTest#getTestName()
	 */
	@Override
	public String getTestName() {
		return "Domino Tower";
	}
	
}

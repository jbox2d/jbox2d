package org.jbox2d.testbed.tests;

import org.jbox2d.common.Mat22;
import org.jbox2d.common.OBBViewportTransform;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;

public class ViewportTest extends SpriteBinding {

	public ViewportTest(TestbedMain parent) {
		super(parent);
	}
	
	public String getName(){
		return "Viewport Test";
	}
	
	@Override
	public void create() {
		super.create();
	}
    
    public int count = 0;
    public Mat22 transform = new Mat22();
    
    public void postStep(){
    	OBBViewportTransform vp = getViewportTransform();
    	
    	//if(count < 300){
    		Mat22.createRotationalTransform(.001f, transform);
        	vp.mulByTransform(transform);
    	/*}else if(count < 600){
    		Mat22.createScaleTransform(20, transform);
    		Mat22 R = vp.getTransform();
    		R.col1.x += (transform.col1.x - R.col1.x) * .02f;
    		R.col1.y += (transform.col1.y - R.col1.y) * .02f;
    		R.col2.x += (transform.col2.x - R.col2.x) * .02f;
    		R.col2.y += (transform.col2.y - R.col2.y) * .02f;
    	}else if(count >= 800){
    		count = 0;
    	}
    	count++;*/
    }
}

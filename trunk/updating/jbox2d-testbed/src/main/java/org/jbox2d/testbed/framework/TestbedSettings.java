/**
 * Created at 1:58:18 PM Jul 17, 2010
 */
package org.jbox2d.testbed.framework;

/**
 * @author Daniel Murphy
 */
public class TestbedSettings {
	public float hz = 60f;
	public int velocityIterations = 8;
	public int positionIterations = 3;
	public boolean drawShapes = true;
	public boolean drawJoints = false;
	public boolean drawAABBs = false;
	public boolean drawPairs = false;
	public boolean drawContactPoints = false;
	public boolean drawContactNormals = false;
	public boolean drawContactForces = false;
	public boolean drawFrictionForces = false;
	public boolean drawCOMs = false;
	public boolean drawStats = true;
	public boolean drawDynamicTree = false;
	public boolean enableWarmStarting = false;
	public boolean enableContinuous = true;
	public boolean pause = false;
	public boolean singleStep = false;
}

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
	public int drawShapes = 1;
	public int drawJoints = 1;
	public int drawAABBs = 0;
	public int drawPairs = 0;
	public boolean drawContactPoints = false;
	public boolean drawContactNormals = false;
	public boolean drawContactForces = false;
	public boolean drawFrictionForces = false;
	public int drawCOMs = 0;
	public boolean drawStats = false;
	public boolean enableWarmStarting = true;
	public boolean enableContinuous = true;
	public boolean pause = false;
	public boolean singleStep = false;
}

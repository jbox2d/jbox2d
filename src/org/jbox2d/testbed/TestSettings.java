/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.gphysics.com
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

public class TestSettings {
    public int hz;

    public int iterationCount;

    public boolean enableWarmStarting;
    public boolean enablePositionCorrection;
    public boolean enableTOI;

	public boolean pause;
	public boolean singleStep;

	public boolean drawShapes;
	public boolean drawJoints;
	public boolean drawCoreShapes;
	public boolean drawOBBs;
	public boolean drawCOMs;
    public boolean drawStats;
    public boolean drawImpulses;
    public boolean drawAABBs;
    public boolean drawPairs;
	public boolean drawContactPoints;
	public boolean drawContactNormals;
	public boolean drawContactForces;
	public boolean drawFrictionForces;

    public TestSettings() {
        hz = 60;
        iterationCount = 10;
        drawStats = true;
        drawAABBs = false;
        drawPairs = false;
        drawShapes = true;
        drawJoints = true;
        drawCoreShapes = false;
        drawContactPoints = false;
        drawContactNormals = false;
        drawContactForces = false;
        drawFrictionForces = false;
        drawOBBs = false;
        drawCOMs = false;
        enableWarmStarting = true;
        enablePositionCorrection = true;
        enableTOI = true;
        pause = false;
        singleStep = false;
    }
}

/**
 * 
 */
package org.jbox2d.dynamics.controllers;

import org.jbox2d.dynamics.Body;

/**
 * @author eric
 *
 */
public class ControllerEdge {
	/** provides quick access to other end of this edge.*/
	public Controller controller;		
	/** the body */
	public Body body;					 
	/** the previous controller edge in the controllers's joint list */
	public ControllerEdge prevBody;		
	/** the next controller edge in the controllers's joint list */
	public ControllerEdge nextBody;		
	/** the previous controller edge in the body's joint list */
	public ControllerEdge prevController; 
	/** the next controller edge in the body's joint list */
	public ControllerEdge nextController;
}

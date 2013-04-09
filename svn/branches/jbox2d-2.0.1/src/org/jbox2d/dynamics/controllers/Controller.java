/**
 * 
 */
package org.jbox2d.dynamics.controllers;

import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.DebugDraw;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.dynamics.World;


/** 
 * Base class for controllers. Controllers are a convience for encapsulating common
 * per-step functionality.
 */
public class Controller {

		/** Controllers override this to implement per-step functionality. */
		public void step(final TimeStep step) {;}

		/** Controllers override this to provide debug drawing. */
		public void draw(DebugDraw debugDraw) {;}

		/** Adds a body to the controller list. */
		public void addBody(Body body) {
			ControllerEdge edge = new ControllerEdge();
			edge.body = body;
			edge.controller = this;
			
			//Add edge to controller list
			edge.nextBody = m_bodyList;
			edge.prevBody = null;
			if(m_bodyList != null) {
				m_bodyList.prevBody = edge;
			}
			m_bodyList = edge;
			++m_bodyCount;

			//Add edge to body list
			edge.nextController = body.m_controllerList;
			edge.prevController = null;
			if(body.m_controllerList != null)
				body.m_controllerList.prevController = edge;
			body.m_controllerList = edge;
		}

		/** Removes a body from the controller list. */
		public void removeBody(Body body) {
			//Assert that the controller is not empty
			assert(m_bodyCount>0);

			//Find the corresponding edge
			ControllerEdge edge = m_bodyList;
			while(edge !=  null && edge.body!=body) {
				edge = edge.nextBody;
			}

			//Assert that we are removing a body that is currently attached to the controller
			assert(edge!=null);

			//Remove edge from controller list
			if(edge.prevBody != null) {
				edge.prevBody.nextBody = edge.nextBody;
			}
			if(edge.nextBody != null) {
				edge.nextBody.prevBody = edge.prevBody;
			}
			if(edge == m_bodyList) {
				m_bodyList = edge.nextBody;
			}
			--m_bodyCount;

			//Remove edge from body list
			if(edge.prevController != null)
				edge.prevController.nextController = edge.nextController;
			if(edge.nextController != null)
				edge.nextController.prevController = edge.prevController;
			if(edge == body.m_controllerList)
				body.m_controllerList = edge.nextController;

		}

		/** Removes all bodies from the controller list. */
		public void clear() {
			while(m_bodyList != null)
			{
				ControllerEdge edge = m_bodyList;

				//Remove edge from controller list
				m_bodyList = edge.nextBody;

				//Remove edge from body list
				if(edge.prevController != null)
					edge.prevController.nextController = edge.nextController;
				if(edge.nextController != null)
					edge.nextController.prevController = edge.prevController;
				if(edge == edge.body.m_controllerList)
					edge.body.m_controllerList = edge.nextController;

			}

			m_bodyCount = 0;
		}

		/** Get the next controller in the world's body list. */
		public Controller getNext() {
			return m_next;
		}
		
		/** Get the parent world of this body. */
		public World getWorld() {
			return m_world;
		}

		/** Get the attached body list */
		public ControllerEdge getBodyList() {
			return m_bodyList;
		}

		public World m_world;

		protected ControllerEdge m_bodyList;
		protected int m_bodyCount;

		protected Controller(ControllerDef def) {
			m_world= null;
			m_bodyList = null;
			m_bodyCount = 0;
			m_prev = null;
			m_next = null;
		}			

		public Controller m_prev;
		public Controller m_next;

}

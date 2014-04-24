/*******************************************************************************
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
package org.jbox2d.testbed.framework;

import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Vector;

import javax.swing.DefaultComboBoxModel;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.common.IViewportTransform;
import org.jbox2d.testbed.framework.j2d.DebugDrawJ2D;
import org.jbox2d.dynamics.Body;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
/**
 * Model for the testbed
 * 
 * @author Daniel
 */
public class SelectionManager {
	
	private static final Logger log = LoggerFactory.getLogger(SelectionManager.class);
	/*
	private final DefaultComboBoxModel tests = new DefaultComboBoxModel();
	private final TestbedSettings settings = new TestbedSettings();
	private DebugDrawJ2D draw;
	private TestbedTest test;

	private final boolean[] keys = new boolean[512];
	private final boolean[] codedKeys = new boolean[512];
	private float calculatedFps;
	private int currTestIndex = -1;
	private TestbedTest runningTest;
	private List<String> implSpecificHelp;
	private TestbedPanel panel;
	private WorldCreator worldCreator = new DefaultWorldCreator();
	*/
	public LinkedList<ISelectable> m_selectedObjects;

	private static SelectionManager m_instance = null;
	protected SelectionManager() {
		m_selectedObjects = new LinkedList<ISelectable>();
		
	}

	public static SelectionManager getInstance() {
		if(m_instance == null) {
			m_instance = new SelectionManager();
		}
		return m_instance;
	}

	
	// clear existing selection and select obj
	public boolean select(ISelectable obj) {
		if (m_selectedObjects.contains(obj)){
			log.debug("Tried to select already selected object!!!!");
			return false;
		}
		
		Iterator<ISelectable> itr =  m_selectedObjects.iterator();
		
		while(itr.hasNext()){
			ISelectable o = itr.next();
			o.setSelect(false);
		}
		
		m_selectedObjects.clear();
		
		obj.setSelect(true);
		m_selectedObjects.add(obj);
		return true;
	}
	
	public boolean select(ISelectable obj, boolean bExtendSelection) {
		if (m_selectedObjects.contains(obj)){
			log.debug("Tried to select already selected object!!!!");
			return false;
		}
		
		
		if (!bExtendSelection)
			return this.select(obj);
		else
		{
			// append obj into the selection
			obj.setSelect(true);
			m_selectedObjects.add(obj);
			return true;
		}
	}
	
	
	public boolean deselect(ISelectable obj){
		if (!m_selectedObjects.contains(obj)){
			log.debug("Tried to deselect an object that was not selected!!!!");
			return false;
		}
		obj.setSelect(false);
		return m_selectedObjects.remove(obj);
	}
	
	public void deselectAll(){
		m_selectedObjects.clear();
	}
	
	
}

/*******************************************************************************
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
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
package org.jbox2d.testbed.framework.j2d.custompanels;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.ListCellRenderer;
import javax.swing.SwingConstants;
import javax.swing.border.EtchedBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import org.jbox2d.testbed.framework.TestbedController;
import org.jbox2d.testbed.framework.TestbedModel;
import org.jbox2d.testbed.framework.TestbedModel.ListItem;
import org.jbox2d.testbed.framework.TestbedSetting;
import org.jbox2d.testbed.framework.TestbedSetting.SettingType;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;
import org.jbox2d.testbed.framework.j2d.TestbedSidePanel;

/**
 * The testbed side panel. Facilitates test and setting changes.
 * 
 * @author Daniel Murphy
 */
@SuppressWarnings("serial")
public class VertexTestPanel extends TestbedSidePanel {

	private JButton pauseButton = new JButton("Pause");
	private JButton stepButton = new JButton("Step");
	private JButton resetButton = new JButton("Reset");
	private JButton quitButton = new JButton("Quit");

	public JButton saveButton = new JButton("Save");
	public JButton loadButton = new JButton("Load");
	
	public VertexTestPanel(TestbedModel argModel, TestbedController argController) {
		super(argModel, argController);
		//initComponents();
	}
	private JPanel getAlgorithmPanel() {
		String [] MSTALGORITHMS = { "Kruskal", "Prim" };
		JPanel algoPanel = new JPanel();
		JLabel text = new JLabel("Minimum Spanning Tree Algorithm");
		
		JComboBox theCombobox = new JComboBox(MSTALGORITHMS);
		
		theCombobox.addActionListener(this);

		algoPanel.add(text);
		algoPanel.add(theCombobox);
		
		return algoPanel;
	}
	
	private JPanel getAnotherPanel() {
		String [] MSTALGORITHMS = { "Karl", "Jaehee" };
		JPanel anotherPanel = new JPanel();
		JLabel text = new JLabel("Minimum Spanning Tree Algorithm");
		
		JComboBox theCombobox = new JComboBox(MSTALGORITHMS);
		
		theCombobox.addActionListener(this);

		anotherPanel.add(text);
		anotherPanel.add(theCombobox);
		
		return anotherPanel;
	}
	
	
	public Component [] createSubPanels() {
		Component [] panels = new Component[2];
		panels[0] = getAlgorithmPanel();
		panels[1] = getAnotherPanel();
		
		
		add(panels[0], "Center");
		add(panels[1], "South");
		
		return panels;
	}
	
	
	/*
	public void initComponents() {
		setLayout(new BorderLayout());
		setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));

		//TestbedSettings settings = model.getSettings();

		JPanel top = new JPanel();
		top.setLayout(new GridLayout(0, 1));
		top.setBorder(BorderFactory.createCompoundBorder(new EtchedBorder(EtchedBorder.LOWERED),
				BorderFactory.createEmptyBorder(10, 10, 10, 10)));
		tests = createComboboxForTests();

		top.add(new JLabel("Choose a test:"));
		top.add(tests);

		//addSettings(top, settings, SettingType.DRAWING);

		add(top, "North");

		JPanel algorithmPanel = getAlgorithmPanel();
		add(algorithmPanel, "Center");
		
		
	}
	*/
	/*
	public void addListeners() {
		super.addListeners();
	}
	*/



	public void stateChanged(ChangeEvent e) {
		super.stateChanged(e);
	}

	public void actionPerformed(ActionEvent e) {
		super.actionPerformed(e);
	}
}

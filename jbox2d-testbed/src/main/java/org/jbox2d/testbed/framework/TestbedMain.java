/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DANIEL MURPHY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
/**
 * Created at 4:23:48 PM Jul 17, 2010
 */
package org.jbox2d.testbed.framework;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSlider;
import javax.swing.UIManager;
import javax.swing.border.EtchedBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import org.jbox2d.testbed.tests.TestList;

/**
 * @author Daniel Murphy
 */
@SuppressWarnings("serial")
public class TestbedMain extends JFrame {
	
	private TestPanel panel;
	private int currTestIndex;
	private SidePanel side;
	
	public TestbedMain(){
		super("JBox2D Testbed");
		setLayout(new BorderLayout());
		TestbedSettings s = new TestbedSettings();
		panel = new TestPanel(s);
		
		panel.addKeyListener(new KeyListener() {
			
			@Override
			public void keyTyped(KeyEvent e) {}
			
			@Override
			public void keyReleased(KeyEvent e) {}
			
			@Override
			public void keyPressed(KeyEvent e) {
				if(e.getKeyChar() == '['){
					lastTest();
				}
				else if(e.getKeyChar() == ']'){
					nextTest();
				}
			}
		});
		
		add(panel, "Center");
		side = new SidePanel(s);
		side.setMain(this);
		add(new JScrollPane(side), "East");   
		pack();
		
		setVisible(true);
		setDefaultCloseOperation(3);
		currTestIndex = 0;
		panel.changeTest(TestList.tests.get(0));
	}
	
	public void nextTest(){
		int index = currTestIndex + 1;
		index %= TestList.tests.size();
		side.tests.setSelectedIndex(index);
	}
	
	public void lastTest(){
		int index = currTestIndex - 1;
		if(index < 0){
			index += TestList.tests.size();
		}
		side.tests.setSelectedIndex(index);
	}
	
	public void resetTest(){
		panel.resetTest();
	}
	
	public void testChanged(int argNew){
		if(argNew == -1){
			return;
		}
		currTestIndex = argNew;
		panel.changeTest(TestList.tests.get(argNew));
		panel.grabFocus();
	}
	
	public static void main(String[] args){
		try{
			UIManager.setLookAndFeel("com.sun.java.swing.plaf.nimbus.NimbusLookAndFeel");
		}catch(Exception e){}
		
		new TestbedMain();
		//test();
	}
	
//	public static void test(){
//		World w = new World(new Vec2(), true);
//		
//		for (int i = 0; i < 2; i++)
//	      {
////	         CircleShape circleShape = new CircleShape();
////	         circleShape.m_radius = 1;
////	         Shape shape = circleShape;
//	         PolygonShape polygonShape = new PolygonShape();
//	         polygonShape.setAsBox(1, 1);
//	         Shape shape = polygonShape;
//
//	         BodyDef bodyDef = new BodyDef();
//	         bodyDef.type = BodyType.DYNAMIC;
//	         bodyDef.position.set(5 * i, 0);
//	         bodyDef.angle = (float) (Math.PI / 4 * i);
//	         bodyDef.allowSleep = false;
//	         Body body = w.createBody(bodyDef);
//	         body.createFixture(shape, 5.0f);
//	         
//	         body.applyForce(new Vec2(-10000 * (i - 1), 0), new Vec2());
//	      }
//		
//		for(int i=0; i<100; i++){
//			w.step(.01f,10, 10);
//		}
//	}
}


/**
 * quick hackup of a side panel
 * @author Daniel Murphy
 */
@SuppressWarnings("serial")
class SidePanel extends JPanel implements ChangeListener, ActionListener{
	
	final TestbedSettings settings;
	TestbedMain main;
	
	private JSlider hz;
	private JLabel hzText;
	private JSlider pos;
	private JLabel posText;
	private JSlider vel;
	private JLabel velText;
	
	public JComboBox tests;
	
	
	private JButton pauseButton = new JButton("Pause");
	private JButton stepButton = new JButton("Step");
	private JButton resetButton = new JButton("Reset");
	private JButton quitButton = new JButton("Quit");
	
	static String[] checkboxLabels = {
		"Warm Starting", "Continuous Collision",
		"Draw Shapes", "Draw Joints", "Draw AABBs", 
		"Draw Pairs", "Draw Contact Points",
		"Draw Contact Normals", "Draw Contact Forces",
		"Draw Friction Forces", "Draw Center of Mass",
		"Draw Stats", "Draw Help", "Draw Dynamic Tree"
	};
	
	public SidePanel(TestbedSettings argSettings){
		settings = argSettings;
		initComponents();
		addListeners();
	}
	
	public void setMain(TestbedMain argMain){
		main = argMain;
	}
	
	public void initComponents(){
		setLayout(new BorderLayout());
		setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));
		
		
		JPanel top = new JPanel();
		top.setLayout(new GridLayout(0, 1));
		top.setBorder(BorderFactory.createCompoundBorder(new EtchedBorder(EtchedBorder.LOWERED), BorderFactory.createEmptyBorder(10, 10, 10, 10)));
		String[] names = new String[TestList.tests.size()];
		for(int i=0; i<names.length; i++){
			names[i] = TestList.tests.get(i).getTestName();
		}
		tests = new JComboBox(names);
		tests.setMaximumSize(new Dimension(250, 20));
		tests.addActionListener(this);
		JPanel testsp = new JPanel();
		testsp.setLayout(new GridLayout(1, 2));
		testsp.add(new JLabel("Choose a test:"));
		testsp.add(tests);
		
		top.add(tests);
		hz = new JSlider(1, 400, (int)settings.hz);
		hz.setMaximumSize(new Dimension(200, 20));
		hz.addChangeListener(this);
		hzText = new JLabel("Hz: "+(int)settings.hz);
		top.add(hzText);
		top.add(hz);
		
		pos = new JSlider(0, 100, (int)settings.positionIterations);
		pos.setMaximumSize(new Dimension(200, 20));
		pos.addChangeListener(this);
		posText = new JLabel("Pos iters: "+(int)settings.positionIterations);
		top.add(posText);
		top.add(pos);
		
		vel = new JSlider(1, 100, (int)settings.velocityIterations);
		vel.setMaximumSize(new Dimension(200, 20));
		vel.addChangeListener(this);
		velText = new JLabel("Vel iters: "+(int)settings.velocityIterations);
		top.add(velText);
		top.add(vel);
		
		add(top, "North");
		
		JPanel middle = new JPanel();
		middle.setLayout(new GridLayout(0, 1));
		middle.setBorder(BorderFactory.createCompoundBorder(new EtchedBorder(EtchedBorder.LOWERED), BorderFactory.createEmptyBorder(5, 10, 5, 10)));
		for(int i=0; i<checkboxLabels.length; i++){
			String s = checkboxLabels[i];
			
			JCheckBox box = new JCheckBox(s);
			box.putClientProperty("index", i);
//			"Warm Starting", "Continuous Collision",
//			"Draw Shapes", "Draw Joints", "Draw AABBs", 
//			"Draw Pairs", "Draw Contact Points",
//			"Draw Contact Normals", "Draw Contact Forces",
//			"Draw Friction Forces", "Draw Center of Mass",
//			"Draw Stats"
			boolean tf = false;
			switch(i){
				case 0:
					tf = settings.enableWarmStarting;
					break;
				case 1:
					tf = settings.enableContinuous;
					break;
				case 2:
					tf = settings.drawShapes;
					break;
				case 3:
					tf = settings.drawJoints;
					break;
				case 4:
					tf = settings.drawAABBs;
					break;
				case 5:
					tf = settings.drawPairs;
					break;
				case 6:
					tf = settings.drawContactPoints;
					break;
				case 7:
					tf = settings.drawContactNormals;
					break;
				case 8:
					tf = settings.drawContactForces;
					break;
				case 9:
					tf = settings.drawFrictionForces;
					break;
				case 10:
					tf = settings.drawCOMs;
					break;
				case 11:
					tf = settings.drawStats;
					break;
				case 12:
					tf = settings.drawHelp;
					break;
				case 13:
					tf = settings.drawDynamicTree;
					break;
				default:
					System.out.println("oh no: "+i);
					tf = false;
			}
			box.setSelected(tf);
			box.addChangeListener(this);
			middle.add(box);
		}
		add(middle, "Center");
		
		Box buttons = Box.createHorizontalBox();
		buttons.add(pauseButton);
		buttons.add(stepButton);
		buttons.add(resetButton);
		buttons.add(quitButton);
		add(buttons, "South");
	}
	
	public void addListeners(){
		pauseButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				if(settings.pause){
					settings.pause = false;
					pauseButton.setText("Pause");
				}else{
					settings.pause = true;
					pauseButton.setText("Resume");
				}
			}
		});
		
		stepButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				settings.singleStep = true;
				if(!settings.pause){
					settings.pause = true;
					pauseButton.setText("Resume");
				}
			}
		});
		
		resetButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				main.resetTest();
			}
		});
		
		quitButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				System.exit(0);
			}
		});
	}

	/**
	 * @see javax.swing.event.ChangeListener#stateChanged(javax.swing.event.ChangeEvent)
	 */
	public void stateChanged(ChangeEvent e) {
		if(e.getSource() instanceof JCheckBox){
			JCheckBox box = (JCheckBox) e.getSource();
			
			int i = (Integer)box.getClientProperty("index");
			boolean tf = box.isSelected();
			switch(i){
				case 0:
					settings.enableWarmStarting = tf;
					break;
				case 1:
					settings.enableContinuous = tf;
					break;
				case 2:
					settings.drawShapes = tf;
					break;
				case 3:
					settings.drawJoints = tf;
					break;
				case 4:
					settings.drawAABBs = tf;
					break;
				case 5:
					settings.drawPairs = tf;
					break;
				case 6:
					settings.drawContactPoints = tf;
					break;
				case 7:
					settings.drawContactNormals = tf;
					break;
				case 8:
					settings.drawContactForces = tf;
					break;
				case 9:
					settings.drawFrictionForces = tf;
					break;
				case 10:
					settings.drawCOMs = tf;
					break;
				case 11:
					settings.drawStats = tf;
					break;
				case 12:
					settings.drawHelp = tf;
					break;
				case 13:
					settings.drawDynamicTree = tf;
					break;
				default:
					System.out.println("oh no: "+i);
			}
		}else if(e.getSource() instanceof JSlider){
			JSlider s = (JSlider) e.getSource();
			
			if(s == hz){
				settings.hz = s.getValue();
				hzText.setText("Hz: "+s.getValue());
			}else if( s == pos){
				settings.positionIterations = s.getValue();
				posText.setText("Pos iters: "+s.getValue());
			}else if(s == vel){
				settings.velocityIterations = s.getValue();
				velText.setText("Vel iters: "+s.getValue());
			}
		}
	}

	/**
	 * @see java.awt.event.ActionListener#actionPerformed(java.awt.event.ActionEvent)
	 */
	public void actionPerformed(ActionEvent e) {		
		main.testChanged(tests.getSelectedIndex());
	}
}

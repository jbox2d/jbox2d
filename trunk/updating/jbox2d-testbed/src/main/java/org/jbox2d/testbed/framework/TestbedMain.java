/**
 * Created at 4:23:48 PM Jul 17, 2010
 */
package org.jbox2d.testbed.framework;

import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSlider;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

/**
 * @author Daniel Murphy
 */
public class TestbedMain extends JFrame {
	
	TestPanel panel;
	
	public TestbedMain(){
		super("JBox2D Testbed");
		setLayout(new BorderLayout());
		TestbedSettings s = new TestbedSettings();
		panel = new TestPanel(s);
		add(panel, "Center");
		add(new JScrollPane(new SidePanel(s,this)), "East");
		pack();
		setVisible(true);
		setDefaultCloseOperation(3);
		panel.changeTest(TestList.tests.get(0));
	}
	
	public void testChanged(TestbedTest argNew){
		panel.changeTest(argNew);
	}
	
	public static void main(String[] args){
		new TestbedMain();
	}
}


/**
 * quick hackup of a side panel
 * @author Daniel Murphy
 */
class SidePanel extends JPanel implements ChangeListener, ActionListener{
	
	final TestbedSettings settings;
	final TestbedMain main;
	
	private JSlider hz;
	private JLabel hzText;
	private JSlider pos;
	private JLabel posText;
	private JSlider vel;
	private JLabel velText;
	
	private JComboBox tests;
	
	static String[] checkboxLabels = {
		"Warm Starting", "Continuous Collision",
		"Draw Shapes", "Draw Joints", "Draw AABBs", 
		"Draw Pairs", "Draw Contact Points",
		"Draw Contact Normals", "Draw Contact Forces",
		"Draw Friction Forces", "Draw Center of Mass",
		"Draw Stats"
	};
	
	public SidePanel(TestbedSettings argSettings, TestbedMain argMain){
		settings = argSettings;
		main = argMain;
		initComponents();
	}
	
	public void initComponents(){
		setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
		
		add(Box.createVerticalGlue());
		
		String[] names = new String[TestList.tests.size()];
		for(int i=0; i<names.length; i++){
			names[i] = TestList.tests.get(i).getTestName();
		}
		tests = new JComboBox(names);
		tests.addActionListener(this);
		Box tbox = Box.createHorizontalBox();
		tbox.add(new JLabel("Choose a test:"));
		tbox.add(tests);
		
		add(tbox);
		
		add(Box.createVerticalGlue());
		
		hz = new JSlider(1, 200, (int)settings.hz);
		hz.setSize(100, 10);
		hz.addChangeListener(this);
		hzText = new JLabel("Hz: "+(int)settings.hz);
		add(hzText);
		add(hz);
		
		pos = new JSlider(1, 100, (int)settings.positionIterations);
		pos.setSize(100, 10);
		pos.addChangeListener(this);
		posText = new JLabel("Pos iters: "+(int)settings.positionIterations);
		add(posText);
		add(pos);
		
		vel = new JSlider(1, 100, (int)settings.velocityIterations);
		vel.addChangeListener(this);
		velText = new JLabel("Vel iters: "+(int)settings.velocityIterations);
		add(velText);
		add(vel);
		
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
				default:
					System.out.println("oh no");
					tf = false;
			}
			box.setSelected(tf);
			box.addChangeListener(this);
			add(box);
		}
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
				default:
					System.out.println("oh no");
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
		main.testChanged( TestList.tests.get(tests.getSelectedIndex()));
	}
}
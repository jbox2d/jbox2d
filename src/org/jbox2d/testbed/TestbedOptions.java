package org.jbox2d.testbed;

import java.lang.reflect.Field;
import java.util.ArrayList;

import javax.vecmath.Color3f;

import processing.core.PApplet;

import org.jbox2d.common.Vec2;

public class TestbedOptions {
	static public float checkboxSize = 15.0f;
	static public float padding = 5.0f;
	static public float sliderWidth = 200.0f;
	
	public float borderWidth = 50.0f;
	
	public TestbedMain p;
	public ArrayList<Checkbox> checkBoxes;
	public ArrayList<SliderInt> sliderInts;
	public String titleString = "*** TESTBED OPTIONS - press 'o' to return to test ***";
	public TestSettings settings;
	
	public TestbedOptions(TestbedMain _p) {
		p = _p;
		checkBoxes = new ArrayList<Checkbox>();
		sliderInts = new ArrayList<SliderInt>();
	}

	public void initialize(AbstractExample test) {
		settings = test.settings;
		checkBoxes.clear();
		sliderInts.clear();
		try{
			Class<?> myClass = settings.getClass();
			Field[] fields = myClass.getFields();
			Vec2 pos = new Vec2(borderWidth+padding,borderWidth+2*padding + 2*checkboxSize);
			for (int i=0; i<fields.length; ++i) {
				//System.out.println(fields[i].getName() + " " +fields[i].getType().getCanonicalName());
				if (fields[i].getType().getCanonicalName().equals("boolean")) {
					String fieldName = fields[i].getName();
					boolean initialV = fields[i].getBoolean(settings);
					//System.out.println(initialV);
					Checkbox myCheck = new Checkbox(pos, fieldName, initialV, fields[i]);
					checkBoxes.add(myCheck);
					pos.y += checkboxSize+padding;
					if (pos.y +checkboxSize+padding> p.height - borderWidth) {
						pos.y = borderWidth+2*padding + 2*checkboxSize;
						pos.x += (p.width - 2*borderWidth) * .5f;
					}
				} else if (fields[i].getType().getCanonicalName().equals("int")) {
					String fieldName = fields[i].getName();
					int initialV = fields[i].getInt(settings);
					//System.out.println(initialV);
					SliderInt mySlider = new SliderInt(pos,sliderWidth,fieldName,initialV,1,5*initialV,fields[i]);
					sliderInts.add(mySlider);
					pos.y += 2 * (checkboxSize+padding);
					if (pos.y +checkboxSize+padding> p.height - borderWidth) {
						pos.y = borderWidth+2*padding + 2*checkboxSize;
						pos.x += (p.width - 2*borderWidth) * .5f;
					}
				}
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	public void handleOptions() {
		p.fill(20,20,110,255);
		p.stroke(255);
		p.rect(borderWidth,borderWidth,p.width-2*borderWidth,p.height-2*borderWidth);
		ProcessingDebugDraw g = (ProcessingDebugDraw)(TestbedMain.currentTest.m_debugDraw);
		g.drawString(borderWidth+padding, borderWidth+padding+AbstractExample.textLineHeight, titleString, new Color3f(255,255,255));
		for (int i=0; i<checkBoxes.size(); ++i) {
			checkBoxes.get(i).process();
			checkBoxes.get(i).draw();
		}
		for (int i=0; i<sliderInts.size(); ++i) {
			sliderInts.get(i).process();
			sliderInts.get(i).draw();
		}
		
	}
	
	class Checkbox {
		public String label;
		public boolean value;
		public Vec2 position;
		/** The value the checkbox controls */
		public Field attachedValue;
		
		public Checkbox(Vec2 _position, String _label, boolean initialValue, Field _attached) {
			position = _position.clone();
			label = _label;
			value = initialValue;
			attachedValue = _attached;
		}
		
		public void process() {
			// Check if mouse clicked inside box
			if (p.mousePressed && !p.pmousePressed && isMouseOver()) {
				// Change the value and set the field
				try{
					value = !value;
					attachedValue.setBoolean(TestbedMain.currentTest.settings, value);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		}
		
		private boolean isMouseOver() {
			return (p.mouseX > position.x && p.mouseX < position.x+checkboxSize &&
					p.mouseY > position.y && p.mouseY < position.y+checkboxSize);
		}
		
		public void draw() {
			if (isMouseOver()) {
				p.fill(155,155,155,200);
			} else {
				p.fill(100,100,100,200);
			}
			p.stroke(0);
			p.rect(position.x,position.y,checkboxSize,checkboxSize);
			if (value) {
				p.fill(0,0,0);
				p.noStroke();
				p.ellipse(position.x+checkboxSize*.5f, position.y+checkboxSize*.5f, checkboxSize*.7f, checkboxSize*.7f);
			}
			ProcessingDebugDraw g = (ProcessingDebugDraw)(TestbedMain.currentTest.m_debugDraw);
			int lineHeight = AbstractExample.textLineHeight;
			g.drawString((int)(position.x + checkboxSize + padding*2), (int)(position.y + .5f*(checkboxSize+lineHeight)), label, new Color3f(255,255,255));
		}
	}
	
	class SliderInt {
		public String label;
		public int value;
		public int minValue, maxValue;
		public float width;
		public Vec2 position;
		/** The value the checkbox controls */
		public Field attachedValue;
		
		public SliderInt(Vec2 _position, float _width, String _label, int initialValue, int _minValue, int _maxValue, Field _attached) {
			position = _position.clone();
			label = _label;
			value = initialValue;
			minValue = _minValue;
			maxValue = _maxValue;
			attachedValue = _attached;
			width = _width;
		}
		
		public void process() {
			if (p.mousePressed && isMouseOver()) {
				// Change the value and set the field
				try{
					value = PApplet.floor(PApplet.map(p.mouseX, position.x, position.x+width, minValue, maxValue));
					attachedValue.setInt(TestbedMain.currentTest.settings, value);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		}
		
		private boolean isMouseOver() {
			return (p.mouseX > position.x && p.mouseX < position.x+width &&
					p.mouseY > position.y && p.mouseY < position.y+checkboxSize);
		}
		
		public void draw() {
			if (isMouseOver()) {
				p.fill(125,125,125,220);
			} else {
				p.fill(100,100,100,200);
			}
			p.stroke(0);
			p.rect(position.x,position.y,width,checkboxSize);
			p.ellipse(PApplet.map(value,minValue,maxValue,position.x,position.x+width),
					  position.y + checkboxSize*.5f, .7f*checkboxSize, .7f*checkboxSize);
			
			ProcessingDebugDraw g = (ProcessingDebugDraw)(TestbedMain.currentTest.m_debugDraw);
			int lineHeight = AbstractExample.textLineHeight;
			g.drawString( (position.x + padding), (position.y + (checkboxSize+lineHeight)), label+": "+value, new Color3f(255,255,255));
		}
	}

}

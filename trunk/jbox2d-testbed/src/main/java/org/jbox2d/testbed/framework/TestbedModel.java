package org.jbox2d.testbed.framework;

import javax.swing.DefaultComboBoxModel;

/**
 * 
 * @author Daniel
 */
public class TestbedModel {

  private final DefaultComboBoxModel tests;
  private final TestbedSettings settings;
	
	public TestbedModel(){
	    settings = new TestbedSettings();
	    tests = new DefaultComboBoxModel();
	}
	
	public void addTest(TestbedTest argTest){
	    tests.addElement(new ListItem(argTest));
	}
	
	public void addCategory(String argName){
	    tests.addElement(new ListItem(argName));
	}
	
	public TestbedTest getTestAt(int argIndex){
	    ListItem item = (ListItem) tests.getElementAt(argIndex);
	    if(item.isCategory()){
	        return null;
	    }
	    return item.test;
	}
	
	public boolean isTestAt(int argIndex){
	    ListItem item = (ListItem) tests.getElementAt(argIndex);
      return !item.isCategory();
	}
	
	public void clearTestList(){
	    tests.removeAllElements();
	}
	
	public int getTestsSize(){
	    return tests.getSize();
	}
	
	public DefaultComboBoxModel getComboModel(){
	    return tests;
	}
	
	public TestbedSettings getSettings(){
	    return settings;
	}
	
	public class ListItem {
		public String category;
		public TestbedTest test;
		
		public ListItem(String argCategory){
		    category = argCategory;
		}
		
		public ListItem(TestbedTest argTest){
		    test = argTest;
		}
		
		public boolean isCategory(){
		    return category != null;
		}
	}
}

package org.jbox2d.collision;


public class DynamicTreeNode {
	/**
	 * This is the fattened AABB
	 */
	public final AABB aabb = new AABB();
	
	public Object userData;
	
	protected DynamicTreeNode parent;
	
	protected DynamicTreeNode child1;
	protected DynamicTreeNode child2;
	
	public final boolean isLeaf(){
		return child1 == null;
	}
	
	public DynamicTreeNode(){}
}

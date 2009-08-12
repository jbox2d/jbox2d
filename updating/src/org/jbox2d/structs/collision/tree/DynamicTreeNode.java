package org.jbox2d.structs.collision.tree;

import org.jbox2d.collision.AABB;

public class DynamicTreeNode {
	/**
	 * This is the fattened AABB
	 */
	public final AABB aabb = new AABB();
	
	public Object userData;
	
	public DynamicTreeNode parent;
	//public DynamicTreeNode next;
	
	public DynamicTreeNode child1;
	public DynamicTreeNode child2;
	
	public final boolean isLeaf(){
		return child1 == null;
	}
	
	public DynamicTreeNode(){}
}

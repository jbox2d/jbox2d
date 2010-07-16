package org.jbox2d.structs.collision.broadphase;

import org.jbox2d.collision.broadphase.DynamicTreeNode;

/**
 * Java note: at the "creation" of each node, a random key is given to
 * that node, and that's what we sort from.
 */
public class Pair implements Comparable<Pair>{
	public DynamicTreeNode proxyA;
	public DynamicTreeNode proxyB;
	
	public int compareTo(Pair pair2) {
		if (this.proxyA.key < pair2.proxyA.key){
			return -1;
		}

		if (this.proxyA.key == pair2.proxyA.key){
			
			if(proxyB.key < pair2.proxyB.key){
				return -1;
			}else{
				if(proxyB.key == pair2.proxyB.key){
					return 0;
				}else{
					return 1;
				}
			}
		}

		return 1;
	}
}

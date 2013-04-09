/**
 * 
 */
package org.jbox2d.testbed.tests.character;

/**
 * @author eric
 *
 */
public class CharacterController {
	private CharacterModel character;
	private boolean up, down, left, right, jump;
	
	public void setCharacter(CharacterModel c) {
		character = c;
	}
	
	public void setJump(boolean tf) {
		jump = tf;
	}
	
	public void setLeft(boolean tf) { left = tf; }
	public void setRight(boolean tf) { right = tf; }
	
	public void step(float dt) {
		character.tryToJump(jump);
		character.moveLeft(left?1.0f:0.0f);
		character.moveRight(right?1.0f:0.0f);
	}
}

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
/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/
 * Box2D homepage: http://www.box2d.org
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
package org.jbox2d.structs.collision;


//FIXME: In the C++ version, this class is a union of
//the key and the features, meaning not that it contains
//both separately, but that the same data can be accessed
//as either.  The key there is 32 bit, and each member of
//features is 8 bits.
//
//We need to figure out if this is a problem or not, because
//I have a feeling that as of right now, key is never being
//set anyways.  Initial examination seems to show that key is
//always zero. [hacked around for the moment]
//
//Also, it might be better performance-wise to pull features
//to a top level class if inner classes have more overhead (check this).

// updated to rev 100

/** Contact ids to facilitate warm starting.*/
public class ContactID {

	/** The features that intersect to form the contact point */
	public final Features features;

	/** The features that intersect to form the contact point */
	public static class Features {
		/** The edge that defines the outward contact normal. */
		public int referenceEdge;
		/** The edge most anti-parallel to the reference edge. */
		public int incidentEdge;
		/** The vertex (0 or 1) on the incident edge that was clipped. */
		public int incidentVertex;
		/** A value of 1 indicates that the reference edge is on shape2. */
		public int flip;

		public Features() {
			referenceEdge = incidentEdge = incidentVertex = flip = 0;
		}

		private Features(final Features f) {
			referenceEdge = f.referenceEdge;
			incidentEdge = f.incidentEdge;
			incidentVertex = f.incidentVertex;
			flip = f.flip;
		}

		private void set(final Features f){
			referenceEdge = f.referenceEdge;
			incidentEdge = f.incidentEdge;
			incidentVertex = f.incidentVertex;
			flip = f.flip;
		}

		private boolean isEqual(final Features f){
			return (referenceEdge==f.referenceEdge &&
					incidentEdge==f.incidentEdge &&
					incidentVertex==f.incidentVertex &&
					flip==f.flip);
		}

		@Override
		public String toString() {
			final String s = "Features: (" + this.flip + " ," + this.incidentEdge + " ," + this.incidentVertex + " ," + this.referenceEdge + ")";
			return s;
		}

	}

	public boolean isEqual(final ContactID cid) {
		return cid.features.isEqual(this.features);
	}

	public ContactID() {
		features = new Features();
	}

	public ContactID(final ContactID c) {
		features = new Features(c.features);
	}

	public void set(final ContactID c){
		features.set(c.features);
	}
	
	/**
	 * zeros out the data
	 */
	public void zero() {
		features.flip = 0;
		features.incidentEdge = 0;
		features.incidentVertex = 0;
		features.referenceEdge = 0;
	}

}

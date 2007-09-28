package collision;

public class ContactID {
	public int key;
	// UNION
	Features features;

	class Features {
		int referenceFace;
		int incidentEdge;
		int incidentVertex;
		int flip;
	}
}

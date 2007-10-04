package collision;

public class ContactID {
    public int key;

    // UNION
    Features features;

    class Features {
        public int referenceFace;

        public int incidentEdge;

        public int incidentVertex;

        public int flip;

        public Features() {
            referenceFace = incidentEdge = incidentVertex = flip = 0;
        }

    }

    public ContactID() {
        key = 0;
        features = new Features();
    }

}

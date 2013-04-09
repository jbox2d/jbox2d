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

        public Features(Features f) {
            referenceFace = f.referenceFace;
            incidentEdge = f.incidentEdge;
            incidentVertex = f.incidentVertex;
            flip = f.flip;
        }

    }

    public ContactID() {
        key = 0;
        features = new Features();
    }

    public ContactID(ContactID c) {
        key = c.key;
        features = new Features(c.features);
    }

}

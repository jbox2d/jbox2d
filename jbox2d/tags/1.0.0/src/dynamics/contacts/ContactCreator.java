package dynamics.contacts;

import collision.Shape;

public interface ContactCreator {

    Contact create(Shape s1, Shape s2);
}

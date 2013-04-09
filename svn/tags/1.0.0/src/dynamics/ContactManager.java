package dynamics;

import collision.PairCallback;
import collision.Shape;
import dynamics.contacts.Contact;
import dynamics.contacts.NullContact;

public class ContactManager extends PairCallback {
    World m_world;

    // This lets us provide broadphase proxy pair user data for
    // contacts that shouldn't exist.
    NullContact m_nullContact;

    ContactManager() {
        m_nullContact = new NullContact();
    }

    public Object PairAdded(Object proxyUserData1, Object proxyUserData2) {
        Shape shape1 = (Shape) proxyUserData1;
        Shape shape2 = (Shape) proxyUserData2;

        Body body1 = shape1.m_body;
        Body body2 = shape2.m_body;

        if (body1.IsStatic() && body2.IsStatic()) {
            return m_nullContact;
        }

        if (shape1.m_body == shape2.m_body) {
            return m_nullContact;
        }

        // Ensure that body2 is dynamic (body1 is static or dynamic).
        if (body2.m_invMass == 0.0f) {
            // b2Swap(shape1, shape2);
            // b2Swap(body1, body2);
            Shape tmps = shape1;
            shape1 = shape2;
            shape2 = tmps;
            Body tmpb = body1;
            body1 = body2;
            body2 = tmpb;
        }

        if (body2.isConnected(body1)) {
            return m_nullContact;
        }

        // Call the factory.
        Contact contact = Contact.Create(shape1, shape2);

        if (contact == null) {
            return m_nullContact;
        }
        else {
            // Insert into the world.
            contact.m_prev = null;
            contact.m_next = m_world.m_contactList;
            if (m_world.m_contactList != null) {
                m_world.m_contactList.m_prev = contact;
            }
            m_world.m_contactList = contact;
            ++m_world.m_contactCount;
        }

        return contact;
    }

    public void PairRemoved(Object pairUserData) {
        Contact c = (Contact) pairUserData;
        if (c != m_nullContact) {
            // Remove from the world.
            if (c.m_prev != null) {
                c.m_prev.m_next = c.m_next;
            }

            if (c.m_next != null) {
                c.m_next.m_prev = c.m_prev;
            }

            if (c == m_world.m_contactList) {
                m_world.m_contactList = c.m_next;
            }

            if (c.GetManifoldCount() > 0) {
                Body body1 = c.m_shape1.m_body;
                Body body2 = c.m_shape2.m_body;

                // Wake up touching bodies.
                body1.wakeUp();
                body2.wakeUp();

                // Disconnect from island graph.
                // Remove from body 1
                if (c.m_node1.prev != null) {
                    c.m_node1.prev.next = c.m_node1.next;
                }

                if (c.m_node1.next != null) {
                    c.m_node1.next.prev = c.m_node1.prev;
                }

                if (c.m_node1 == body1.m_contactList) {
                    body1.m_contactList = c.m_node1.next;
                }

                c.m_node1.prev = null;
                c.m_node1.next = null;

                // Remove from body 2
                if (c.m_node2.prev != null) {
                    c.m_node2.prev.next = c.m_node2.next;
                }

                if (c.m_node2.next != null) {
                    c.m_node2.next.prev = c.m_node2.prev;
                }

                if (c.m_node2 == body2.m_contactList) {
                    body2.m_contactList = c.m_node2.next;
                }

                c.m_node2.prev = null;
                c.m_node2.next = null;
            }

            // Call the factory.
            --m_world.m_contactCount;
        }
    }

    void Collide() {
        for (Contact c = m_world.m_contactList; c != null; c = c.m_next) {
            if (c.m_shape1.m_body.isSleeping()
                    && c.m_shape2.m_body.isSleeping()) {
                continue;
            }

            int oldCount = c.GetManifoldCount();
            c.Evaluate();

            int newCount = c.GetManifoldCount();

            if (oldCount == 0 && newCount > 0) {
                // Connect to island graph.

                Body body1 = c.m_shape1.m_body;
                Body body2 = c.m_shape2.m_body;

                // Connect to body 1
                c.m_node1.contact = c;
                c.m_node1.other = body2;

                c.m_node1.prev = null;
                c.m_node1.next = body1.m_contactList;
                if (c.m_node1.next != null) {
                    c.m_node1.next.prev = c.m_node1;
                }
                body1.m_contactList = c.m_node1;

                // Connect to body 2
                c.m_node2.contact = c;
                c.m_node2.other = body1;

                c.m_node2.prev = null;
                c.m_node2.next = body2.m_contactList;
                if (c.m_node2.next != null) {
                    c.m_node2.next.prev = c.m_node2;
                }
                body2.m_contactList = c.m_node2;
            }
            else if (oldCount > 0 && newCount == 0) {
                // Disconnect from island graph.
                Body body1 = c.m_shape1.m_body;
                Body body2 = c.m_shape2.m_body;

                // Remove from body 1
                if (c.m_node1.prev != null) {
                    c.m_node1.prev.next = c.m_node1.next;
                }

                if (c.m_node1.next != null) {
                    c.m_node1.next.prev = c.m_node1.prev;
                }

                if (c.m_node1 == body1.m_contactList) {
                    body1.m_contactList = c.m_node1.next;
                }

                c.m_node1.prev = null;
                c.m_node1.next = null;

                // Remove from body 2
                if (c.m_node2.prev != null) {
                    c.m_node2.prev.next = c.m_node2.next;
                }

                if (c.m_node2.next != null) {
                    c.m_node2.next.prev = c.m_node2.prev;
                }

                if (c.m_node2 == body2.m_contactList) {
                    body2.m_contactList = c.m_node2.next;
                }

                c.m_node2.prev = null;
                c.m_node2.next = null;
            }
        }
    }
}

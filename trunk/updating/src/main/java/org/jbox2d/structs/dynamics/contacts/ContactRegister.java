package org.jbox2d.structs.dynamics.contacts;

import org.jbox2d.structs.collision.shapes.ShapeType;

public class ContactRegister {
    public ContactCreateFcn createFcn;
    public ContactDestroyFcn destroyFcn;

    public boolean primary;
}

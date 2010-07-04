/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.gphysics.com
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
package dynamics;

import common.Settings;
import common.Vec2;

import dynamics.contacts.NullContact;
import dynamics.contacts.Contact;
import dynamics.contacts.ContactSolver;
import dynamics.joints.Joint;

//Updated to rev. 46 of b2Island.cpp/.h

public class Island {
    public Body[] m_bodies;

    public Contact[] m_contacts;

    public Joint[] m_joints;

    public int m_bodyCount;

    public int m_jointCount;

    public int m_contactCount;

    public int m_bodyCapacity;

    public int m_contactCapacity;

    public int m_jointCapacity;

    public static int m_positionIterationCount = 0;

    public float m_positionError;

    public Island(int bodyCapacity, int contactCapacity, int jointCapacity) {
        m_bodyCapacity = bodyCapacity;
        m_contactCapacity = contactCapacity;
        m_jointCapacity = jointCapacity;
        m_bodyCount = 0;
        m_contactCount = 0;
        m_jointCount = 0;

        m_bodies = new Body[bodyCapacity];
        m_contacts = new Contact[contactCapacity];
        m_joints = new Joint[jointCapacity];

        for (int i = 0; i < contactCapacity; i++) {
            m_contacts[i] = new NullContact();
        }
    }

    void clear() {
        m_bodyCount = 0;
        m_contactCount = 0;
        m_jointCount = 0;
    }

    void solve(TimeStep step, Vec2 gravity) {
        for (int i = 0; i < m_bodyCount; ++i) {
            Body b = m_bodies[i];

            if (b.m_invMass == 0.0f) {
                continue;
            }

//            b.m_linearVelocity.addLocal(b.m_force.clone().mulLocal(b.m_invMass)
//                    .addLocal(gravity).mulLocal(step.dt));
//
//            b.m_angularVelocity += step.dt * b.m_invI * b.m_torque;
//
//            b.m_linearVelocity.mulLocal(b.m_linearDamping);
//            b.m_angularVelocity *= b.m_angularDamping;
//            
            b.m_linearVelocity.x += step.dt * (gravity.x + b.m_invMass * b.m_force.x);
            b.m_linearVelocity.y += step.dt * (gravity.y + b.m_invMass * b.m_force.y);
            b.m_angularVelocity += step.dt * b.m_invI * b.m_torque;

            b.m_linearVelocity.mulLocal(b.m_linearDamping);
            b.m_angularVelocity *= b.m_angularDamping;

            // Store positions for conservative advancement.
            b.m_position0.set(b.m_position);
            b.m_rotation0 = b.m_rotation;
        }

        ContactSolver contactSolver = new ContactSolver(m_contacts,
                m_contactCount);

        // Pre-solve
        contactSolver.preSolve();

        for (int i = 0; i < m_jointCount; ++i) {
            m_joints[i].prepareVelocitySolver();
        }

        // Solve velocity constraints.
        for (int i = 0; i < step.iterations; ++i) {
            contactSolver.solveVelocityConstraints();

            for (int j = 0; j < m_jointCount; ++j) {
                m_joints[j].solveVelocityConstraints(step);
            }
        }

        // Integrate positions.
        for (int i = 0; i < m_bodyCount; ++i) {
            Body b = m_bodies[i];
            if (b.m_invMass == 0.0f) {
                continue;
            }

            //b.m_position.addLocal(b.m_linearVelocity.mul(step.dt));
            b.m_position.x += b.m_linearVelocity.x*step.dt;
            b.m_position.y += b.m_linearVelocity.y*step.dt;
            b.m_rotation += step.dt * b.m_angularVelocity;

            b.m_R.setAngle(b.m_rotation);
        }
        
        for (int i = 0; i < m_jointCount; ++i) {
            m_joints[i].preparePositionSolver();
        }

        // Solve position constraints.
        if (World.ENABLE_POSITION_CORRECTION) {
            // System.out.println("position correcting");
            for (m_positionIterationCount = 0; m_positionIterationCount < step.iterations; ++m_positionIterationCount) {
                // System.out.println(m_positionIterations);
                boolean contactsOkay = contactSolver
                        .solvePositionConstraints(Settings.contactBaumgarte);

                boolean jointsOkay = true;
                for (int i = 0; i < m_jointCount; ++i) {
                    boolean jointOkay = m_joints[i].solvePositionConstraints();
                    jointsOkay = jointsOkay && jointOkay;
                }

                if (contactsOkay && jointsOkay) {
                    break;
                }
            }
        }

        // Post-solve.
        contactSolver.postSolve();

        // Synchronize shapes and reset forces.
        for (int i = 0; i < m_bodyCount; ++i) {
            Body b = m_bodies[i];
            if (b.m_invMass == 0.0f)
                continue;
            b.m_R.setAngle(b.m_rotation);
            b.synchronizeShapes();
            b.m_force.set(0.0f, 0.0f);
            b.m_torque = 0.0f;
        }

    }

    void updateSleep(float dt) {
        float minSleepTime = Float.MAX_VALUE;

        final float linTolSqr = Settings.linearSleepTolerance
                * Settings.linearSleepTolerance;
        final float angTolSqr = Settings.angularSleepTolerance
                * Settings.angularSleepTolerance;

        for (int i = 0; i < m_bodyCount; ++i) {
            Body b = m_bodies[i];
            if (b.m_invMass == 0.0f) {
                continue;
            }

            if ((b.m_flags & Body.e_allowSleepFlag) == 0) {
                b.m_sleepTime = 0.0f;
                minSleepTime = 0.0f;
            }

            if (((b.m_flags & Body.e_allowSleepFlag) == 0)
                    || b.m_angularVelocity * b.m_angularVelocity > angTolSqr
                    || Vec2.dot(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr) {
                b.m_sleepTime = 0.0f;
                minSleepTime = 0.0f;
            }
            else {
                b.m_sleepTime += dt;
                minSleepTime = Math.min(minSleepTime, b.m_sleepTime);
            }
        }

        if (minSleepTime >= Settings.timeToSleep) {
            for (int i = 0; i < m_bodyCount; ++i) {
                Body b = m_bodies[i];
                b.m_flags |= Body.e_sleepFlag;
            }
        }
    }

    void add(Body body) {
        assert m_bodyCount < m_bodyCapacity;
        m_bodies[m_bodyCount++] = body;
    }

    void add(Contact contact) {
        assert (m_contactCount < m_contactCapacity);
        m_contacts[m_contactCount++] = contact.clone();
    }

    void add(Joint joint) {
        assert (m_jointCount < m_jointCapacity);
        m_joints[m_jointCount++] = joint;
    }
}

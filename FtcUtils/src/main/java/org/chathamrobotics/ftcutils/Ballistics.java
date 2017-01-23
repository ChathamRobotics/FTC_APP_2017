package org.chathamrobotics.ftcutils;

/**
 * Created by storm on 1/22/2017.
 */

public final class Ballistics {
    public static final double GRAVITY_ACCELERATION = 9.81;

    /**
     * find the required velocity to fire a projectile at a target
     * @param distance  the horizontal displacement
     * @param height    the vertical displacement
     * @param fireAngle the angle the project is being fired at
     * @return          the required velocity
     */
    public double requiredVelocity(double distance, double height, double fireAngle) {
        return (distance * GRAVITY_ACCELERATION) /
                Math.sqrt(2 * distance * GRAVITY_ACCELERATION * Math.cos(fireAngle) * Math.sin(fireAngle)
                        - 2 * GRAVITY_ACCELERATION * height * Math.pow(Math.cos(fireAngle), 2));
    }
}

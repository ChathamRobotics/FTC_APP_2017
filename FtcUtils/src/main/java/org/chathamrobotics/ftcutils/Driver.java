package org.chathamrobotics.ftcutils;

/**
 * A common driver interface
 */

public interface Driver {
    public void drive(double x, double y, double rotate);
    public void drive(double x, double y, double rotate, double speedModifier);

    public void stop();
}

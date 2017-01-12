package org.chathamrobotics.ftcutils;

/**
 * A common driver interface
 */

public interface Driver {

    /**
     * Drives the robot given the x, y and rotation values
     * @param x         the x value for direction
     * @param y         the y value for direction
     * @param rotate    the rotation value
     */
    public void drive(double x, double y, double rotate);

    /**
     * Drives the robot in the specified direction
     * @param x             the x value for the direction
     * @param y             the y value for the direction
     * @param rotate        the rotation value
     * @param speedModifier the speed modifier
     */
    public void drive(double x, double y, double rotate, double speedModifier);

    /**
     * stops all driving functionality
     */
    public void stop();
}

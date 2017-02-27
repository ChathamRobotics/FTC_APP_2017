package org.firstinspires.ftc.team9853.autonomous;

import org.chathamrobotics.ftcutils.Robot;
import org.chathamrobotics.ftcutils.StoppedException;
import org.firstinspires.ftc.team9853.Robot9853;
import org.firstinspires.ftc.team9853.opmodes.Auto9853;


/**
 * charger autonomous - hits cap ball and shoots
 */
public class AutoModeCharge extends Auto9853 {
//    CONSTANTS     //
//    CONSTRUCTORS  //
    /*
     * Setup OpMode
     * @param {boolean} isRedTeam   Whether the current team is red
     */
    public AutoModeCharge(boolean isRedTeam) {
        super(isRedTeam);
    }

//    METHODS       //

    /**
     * called on start
     */
    public void runRobot() throws StoppedException {
        // Waits a little bit before starting autonomous
        while(robot().doUntil(Robot.AUTO_START_WAIT_TIME)) statusCheck();

        // Drives to the shooting point
        while(robot().driveForwardFor(Robot9853.SENSING_SPEED, 500)) statusCheck();

        while(robot().shootFor(Robot9853.SHOOT_TIME)) statusCheck();
        while(robot().reloadFor(Robot9853.RELOAD_TIME)) statusCheck();
        while(robot().shootFor(Robot9853.SHOOT_TIME)) statusCheck();

        // Drives to center
        while(robot().driveForwardFor(.7, 2200)) statusCheck();
    }
}

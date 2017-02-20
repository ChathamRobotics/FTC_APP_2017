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
    private static final long driveTime = 3000;

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
        while(robot().driveForwardFor(.7, driveTime / 4)) statusCheck();

        // wait
        while(robot().doUntil(1500)) statusCheck();

        // Shoot
        while(robot().shootFor(Robot9853.SHOOT_TIME)) statusCheck();

        // Reload
        while(robot().reloadFor(Robot9853.RELOAD_TIME)) statusCheck();

        // shoot
        while(robot().shootFor(Robot9853.SHOOT_TIME)) statusCheck();

        // Drives to center
        while(robot().driveForwardFor(.7, 3 * driveTime / 4)) statusCheck();
    }
}

package org.firstinspires.ftc.team9853.autonomous;

import org.chathamrobotics.ftcutils.StoppedException;
import org.firstinspires.ftc.team9853.Robot9853;
import org.firstinspires.ftc.team9853.opmodes.Auto;


/**
 * charger autonomous - hits cap ball and shoots
 */
public class AutoModeCharge extends Auto {
//    CONSTANTS     //
    private static final long waitTime = 10000;
    private static final long driveTime = 3000;

//    CONSTRUCTORS  //
    /*
     * Setup OpMode
     * @param {boolean} isRedTeam   Whether the current team is red
     */
    public AutoModeCharge(boolean isRedTeam) {
        this.isRedTeam = isRedTeam;
    }

//    METHODS       //

    /**
     * called on start
     */
    public void runRobot() throws StoppedException {
        robot().changeFront(Robot9853.Side.BACK);

        // Waits a little bit before starting autonomous
        for(long endTime = robot().calcEndTime(waitTime); robot().doUntil(endTime);) {
            statusCheck();
        }

        // Drives to the shooting point
        for(long endTime =  robot().calcEndTime(driveTime / 4); robot().driveForwardFor(.7, endTime);) {
            statusCheck();
        }

        // wait
        for(long endTime = robot().calcEndTime(1500); robot().doUntil(endTime);) {
            statusCheck();
        }

        // Shoot
        for(long endTime = robot().calcEndTime(Robot9853.SHOOT_TIME); robot().shootFor(endTime);) {
            statusCheck();
        }

        // Reload
        for(long endTime = robot().calcEndTime(Robot9853.RELOAD_TIME); robot().reloadFor(endTime);) {
            statusCheck();
        }

        // shoot
        for(long endTime = robot().calcEndTime(Robot9853.SHOOT_TIME); robot().shootFor(endTime);) {
            statusCheck();
        }

        // Drives to center
        for(long endTime = robot().calcEndTime(3 * driveTime / 4); robot().driveForwardFor(.7, endTime);) {
            statusCheck();
        }
    }
}

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
    private static final long driveTime = 2500;
    private static final long shootTime = 500;
    private static final long reloadTime = 2500;

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
        robot.changeFront(Robot9853.Side.BACK);

        // Waits a little bit before starting autonomous
        for(long endTime = robot.calcEndTime(waitTime); robot.doUntil(endTime);) {
            statusCheck();
        }

        // Drives to the shooting point
        for(long endTime =  robot.calcEndTime(3 * driveTime / 4); robot.driveForwardFor(.7, endTime);) {
            statusCheck();
        }

        // Shoots twice
        for(long endTime = robot.calcEndTime(robot.SHOOT_TIME); System.currentTimeMillis() < endTime;) {
            statusCheck();
            robot.shooter.setPower(-.7);
        }
        robot.shooter.setPower(0);

        for(long endTime = robot.calcEndTime(robot.RELOAD_TIME); System.currentTimeMillis() < endTime;) {
            statusCheck();
            robot.setCollectorPower(1);
        }
        robot.setCollectorPower(0);

        for(long endTime = robot.calcEndTime(robot.SHOOT_TIME); System.currentTimeMillis() < endTime;) {
            statusCheck();
            robot.shooter.setPower(-.7);
        }
        robot.shooter.setPower(0);

        // Drives to center
        for(long endTime = robot.calcEndTime(driveTime / 4); robot.driveForwardFor(.7, endTime);) {
            statusCheck();
        }
    }
}

package org.firstinspires.ftc.team9853.autonomous;

import org.chathamrobotics.ftcutils.Robot;
import org.chathamrobotics.ftcutils.StoppedException;
import org.firstinspires.ftc.team9853.opmodes.Auto9853;

/**
 * hits beacons
 */

public class AutoModeDoubleCheck extends Auto9853 {
//    CONSTANTS     //

//    HARDWARE      //

//    INSTANCE      //

//    CONSTRUCTORS  //

    /**
     * Setup OpMode
     * @param isRedTeam   Whether the current team is red
     */
    public AutoModeDoubleCheck(boolean isRedTeam) {
        super(isRedTeam);
    }

//     METHODS       //

    /**
     * called on start
     * @throws StoppedException
     * @throws InterruptedException
     */
    @Override
    public void runRobot() throws StoppedException, InterruptedException {
        boolean hitFirst, hitSecond;

        // Drive to beacon
        while (robot().driveAtAngleWhile(Math.atan2(6/12d, this.isRedTeam ? 5/12d : -5/12d), .5,
                ! robot().isLeftAtLine())) statusCheck();

        hitBeacon();
        hitFirst = isBeaconCaptured();
        backUp();

        // move to next beacon
        while(robot().driveAtAngleFor(Robot.Side.LEFT.angle, .5, 500)) statusCheck();
        while(robot().driveAtAngleWhile(Robot.Side.LEFT.angle, .5, ! robot().isLeftAtLine()))
            statusCheck();

        hitBeacon();
        hitSecond = isBeaconCaptured();
        backUp();

        if(! hitFirst) {
            while(robot().driveAtAngleFor(Robot.Side.RIGHT.angle, .5, 500)) statusCheck();
            while(robot().driveAtAngleWhile(Robot.Side.RIGHT.angle, .5, ! robot().isLeftAtLine()))
                statusCheck();

            hitBeacon();
            backUp();
        }

        if(! hitSecond) {
            while(robot().driveAtAngleFor(Robot.Side.LEFT.angle, .5, 500)) statusCheck();
            while(robot().driveAtAngleWhile(Robot.Side.LEFT.angle, .5, ! robot().isLeftAtLine()))
                statusCheck();

            hitBeacon();
            backUp();
        }
    }

    /**
     * backs away from the beacon
     * @throws StoppedException
     */
    private void backUp() throws StoppedException {
        while(robot().driveAtAngleFor(Robot.Side.BACK.angle, .2, 500)) statusCheck();
    }
}

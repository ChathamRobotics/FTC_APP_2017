package org.firstinspires.ftc.team9853.autonomous;

import org.chathamrobotics.ftcutils.Robot;
import org.chathamrobotics.ftcutils.StoppedException;
import org.firstinspires.ftc.team9853.Robot9853;
import org.firstinspires.ftc.team9853.opmodes.Auto9853;

/**
 * beacon pressing autonomous
 */

public class AutoModeBeacons extends Auto9853 {
//    CONSTANTS     //
//    private static long PRESS_TIME = 1000;

//    HARDWARE      //

//    CONSTRUCTORS  //

    /**
     * Setup OpMode
     * @param isRedTeam   Whether the current team is red
     */
    public AutoModeBeacons(boolean isRedTeam) {
        super(isRedTeam);
    }

//     METHODS       //

    /**
     * Initializes the robot
     */
    @Override
    public void initRobot() {
        super.initRobot();
    }

    /**
     * called on start
     */
    public void runRobot() throws StoppedException, InterruptedException {
        robot().changeFront(Robot.Side.BACK);

        robot().log("WORKING?");

        // Drive to beacon
        robot().log("Drive Angle");
        while (robot().driveAtAngleWhile(this.isRedTeam ? Math.PI/4 : 3 * Math.PI/4, .32,
                ! robot().isLeftAtLine() && ! robot().isCenterAtLine())) statusCheck();

        // correct if at center
        if(robot().isCenterAtLine()) shiftRight();

        // press beacon
        pressBeacon();

        // move to next beacon
        while(robot().driveAtAngleFor(Robot.Side.LEFT.angle, .32, 500)) statusCheck();
        while(robot().driveAtAngleWhile(Robot.Side.LEFT.angle, .32, ! robot().isLeftAtLine()))
            statusCheck();

        // press beacon
        pressBeacon();
    }

    /**
     * press the right beacon button
     * @throws StoppedException
     */
    private void pressBeacon() throws StoppedException{
        boolean atRight = true, hitRight = false;

        /// ensure that is lined up with the right side of the beacon
        shiftRight();

        // drive until the beacon is in range
        while(robot().driveForwardWhile(Robot9853.SENSING_SPEED, ! robot().isBeaconInRange()))
            statusCheck();
        robot().log("Beacon is in range");

        // check right side for correct color
        if(isRedTeam && robot().isRightRed()) hitRight = true;
        else if(! isRedTeam && robot().isRightBlue()) hitRight = true;

        // shift if needed
        if(! hitRight) {
            shiftLeft();
            atRight = false;
        }

        // hit button
        hitButton();

        // hit again if didn't work
        if((atRight && ((isRedTeam && robot().isLeftRed()) || (! isRedTeam && robot().isLeftBlue())))
                || (! atRight && ((isRedTeam && robot().isRightRed()) || (! isRedTeam && robot().isRightBlue()))))
            hitButton();
    }

    public void hitButton() throws StoppedException{
        // hit
        while(robot().driveForwardWhile(Robot9853.SENSING_SPEED, ! robot().isBeaconTouching()))
            statusCheck();

        // back up
        while(robot().driveAtAngleWhile(Robot.Side.BACK.angle, Robot9853.SENSING_SPEED, robot().isBeaconTouching()))
            statusCheck();
    }

    /**
     * lines the robot up with the right side of the beacon
     * @throws StoppedException
     */
    private void shiftRight() throws StoppedException {
        // move right until left line sensor registers
        while (robot().driveAtAngleWhile(Robot.Side.RIGHT.angle, Robot9853.SENSING_SPEED,
                ! robot().isLeftAtLine())) statusCheck();
    }

    /**
     * lines the robot up with the right side of the beacon
     * @throws StoppedException
     */
    private void shiftLeft() throws StoppedException {
        // move left until left center line sensor registers
        while (robot().driveAtAngleWhile(Robot.Side.LEFT.angle, Robot9853.SENSING_SPEED,
                ! robot().isCenterAtLine())) statusCheck();
    }
}

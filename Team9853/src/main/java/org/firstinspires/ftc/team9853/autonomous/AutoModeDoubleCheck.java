package org.firstinspires.ftc.team9853.autonomous;

import org.chathamrobotics.ftcutils.Robot;
import org.chathamrobotics.ftcutils.StoppedException;
import org.firstinspires.ftc.team9853.Robot9853;
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

        robot().changeFront(Robot.Side.BACK);

        // Drive to beacon
        while (robot().driveWithHeadingWhile(this.isRedTeam ? Math.PI/4 : 3 * Math.PI/4, Robot9853.SENSING_SPEED, robot().startingHeading,
                ! robot().isLeftAtLine())) statusCheck();

        pressBeacon();
        hitFirst = isBeaconCaptured();
        backUp();

        // move to next beacon
        while(robot().driveWithHeadingFor(Robot.Side.LEFT.angle, Robot9853.SENSING_SPEED, robot().startingHeading, 500)) statusCheck();
        while(robot().driveWithHeadingWhile(Robot.Side.LEFT.angle, Robot9853.SENSING_SPEED, robot().startingHeading, ! robot().isLeftAtLine()))
            statusCheck();

        shiftRight();

        pressBeacon();
        hitSecond = isBeaconCaptured();
        backUp();

        if(! hitFirst) {
            while(robot().driveWithHeadingWhile(Robot.Side.RIGHT.angle, Robot9853.SENSING_SPEED, robot().startingHeading, ! robot().isCenterAtLine()))
                statusCheck();

            pressBeacon();
            backUp();
        }

        if(! hitSecond) {
            while(robot().driveWithHeadingWhile(Robot.Side.LEFT.angle, Robot9853.SENSING_SPEED, robot().startingHeading, ! robot().isCenterAtLine()))
                statusCheck();

            pressBeacon();
            backUp();
        }
    }

    /**
     * press the right beacon button
     * @throws StoppedException
     */
    private void pressBeacon() throws StoppedException{
        /// ensure that is lined up with the right side of the beacon
        shiftRight();

        // hit button
        hitButton();
    }

    public void hitButton() throws StoppedException{
        // hit
        while(robot().driveWithHeadingWhile(Robot.Side.FRONT.angle, Robot9853.SENSING_SPEED, robot().startingHeading, ! robot().isBeaconTouching()))
            statusCheck();
    }

    /**
     * lines the robot up with the right side of the beacon
     * @throws StoppedException
     */
    private void shiftRight() throws StoppedException {
        // move right until left line sensor registers
        while (robot().driveWithHeadingWhile(Robot.Side.RIGHT.angle, Robot9853.SENSING_SPEED, robot().startingHeading,
                ! robot().isLeftAtLine())) statusCheck();
    }

    /**
     * lines the robot up with the right side of the beacon
     * @throws StoppedException
     */
    private void shiftLeft() throws StoppedException {
        // move left until left center line sensor registers
        while (robot().driveWithHeadingWhile(Robot.Side.LEFT.angle, Robot9853.SENSING_SPEED, robot().startingHeading,
                ! robot().isCenterAtLine())) statusCheck();
    }

    /**
     * backs away from the beacon
     * @throws StoppedException
     */
    private void backUp() throws StoppedException {
        // back up
        while(robot().driveWithHeadingFor(Robot.Side.BACK.angle, Robot9853.SENSING_SPEED, robot().startingHeading, 500))
            statusCheck();
    }
}
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

//    STATEFUL      //
    private long timerEndTime;

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

        robot().logger.info("Right color", robot().rightBeaconSensor.getColorNumber());

        // Drive to beacon
        while (robot().driveWithHeadingWhile(this.isRedTeam ? Math.PI/4 : 3 * Math.PI/4, Robot9853.SENSING_SPEED, robot().startingHeading,
                ! robot().isLeftAtLine() && ! robot().isCenterAtLine())) statusCheck();

        // correct if at center
        if(robot().isCenterAtLine()) shiftRight();

        // press beacon
        pressBeacon();

        while(robot().shootFor(Robot9853.SHOOT_TIME)) statusCheck();
        while(robot().reloadFor(Robot9853.RELOAD_TIME)) statusCheck();
        while(robot().shootFor(Robot9853.SHOOT_TIME));

        // move to next beacon
        while(robot().driveWithHeadingFor(Robot.Side.LEFT.angle, Robot9853.SENSING_SPEED, robot().startingHeading, 500)) statusCheck();
        while(robot().driveWithHeadingWhile(Robot.Side.LEFT.angle, Robot9853.SENSING_SPEED, robot().startingHeading, ! robot().isLeftAtLine()))
            statusCheck();

        // press beacon
        pressBeacon();

        while(robot().driveWithHeadingFor(315 * Math.PI / 180, .75, robot().startingHeading, 3000))
            statusCheck();
    }

    /**
     * press the right beacon button
     * @throws StoppedException
     */
    private void pressBeacon() throws StoppedException{
        /// ensure that is lined up with the right side of the beacon
        shiftRight();

        // drive until the beacon is in range
        while(robot().driveWithHeadingWhile(Robot.Side.FRONT.angle, Robot9853.SENSING_SPEED, robot().startingHeading, ! robot().isBeaconInRange()))
            statusCheck();
        robot().logger.info("Beacon is in range");

        // check right side for correct color
        robot().logger.info("Beacon RightSide color", getRightSideColor());
        if((isRedTeam && robot().isBlue(robot().getRightBeaconColor()))
                || (! isRedTeam && robot().isRed(robot().getRightBeaconColor()))) shiftLeft();

        // hit button
        hitButton();

        // hit again if didn't work
//        if((atRight && ((isRedTeam && robot().isLeftRed()) || (! isRedTeam && robot().isLeftBlue())))
//                || (! atRight && ((isRedTeam && robot().isRightRed()) || (! isRedTeam && robot().isRightBlue()))))
//            hitButton();
    }

    public void hitButton() throws StoppedException{
        while(! robot().driveWithHeading(0, 0, robot().startingHeading)) statusCheck();
        robot().stopDriving();

        // hit
        timerEndTime = System.currentTimeMillis() + 2000;
        while(robot().driveWithHeadingWhile(Robot.Side.FRONT.angle, Robot9853.SENSING_SPEED,
                robot().startingHeading, ! robot().isBeaconTouching()
                        && System.currentTimeMillis() <= timerEndTime)) statusCheck();

        // back up
        while(robot().driveWithHeadingFor(Robot.Side.BACK.angle, Robot9853.SENSING_SPEED, robot().startingHeading, 500))
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

    private int getRightSideColor() {
        if (robot().centerBeaconSensor.getColorNumber() == robot().rightBeaconSensor.getColorNumber()) return robot().centerBeaconSensor.getColorNumber();
        else if(robot().centerBeaconSensor.getColorNumber() == 0) return robot().rightBeaconSensor.getColorNumber();
        else if(robot().rightBeaconSensor.getColorNumber() == 0) return robot().centerBeaconSensor.getColorNumber();
        else return robot().centerBeaconSensor.getColorNumber();
    }

    private int getLeftSideColor() {
        if (robot().centerBeaconSensor.getColorNumber() == robot().leftBeaconSensor.getColorNumber()) return robot().centerBeaconSensor.getColorNumber();
        else if(robot().centerBeaconSensor.getColorNumber() == 0) return robot().leftBeaconSensor.getColorNumber();
        else if(robot().leftBeaconSensor.getColorNumber() == 0) return robot().centerBeaconSensor.getColorNumber();
        else return robot().centerBeaconSensor.getColorNumber();
    }
}

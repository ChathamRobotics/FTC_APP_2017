package org.firstinspires.ftc.team9853.autonomous;

import org.chathamrobotics.ftcutils.Robot;
import org.chathamrobotics.ftcutils.StoppedException;
import org.firstinspires.ftc.team9853.opmodes.Auto9853;

/**
 * beacon pressing autonomous
 */

public class AutoModeBeacons extends Auto9853 {
//    CONSTANTS     //

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
        // Drive to beacon
        while (robot().driveAtAngleWhile(Math.atan2(6/12d, this.isRedTeam ? 5/12d : -5/12d), .5,
                ! robot().isLeftAtLine())) statusCheck();

        // press beacon
        pressBeacon();

        // move to next beacon
        while(robot().driveAtAngleFor(Robot.Side.LEFT.angle, .5, 500)) statusCheck();
        while(robot().driveAtAngleWhile(Robot.Side.LEFT.angle, .5, ! robot().isLeftAtLine()))
            statusCheck();

        // press beacon
        pressBeacon();
    }

    /**
     * press the right beacon button
     * @throws StoppedException
     */
    private void pressBeacon() throws StoppedException{
        // drive along line
        while(robot().driveForwardWhile(.3, ! robot().isBeaconInRange())) {
            statusCheck();

            if(! robot().isLeftAtLine()) robot().log("Lost Line");
        }

        // line up with correct color
        if(!(isRedTeam && robot().isBeaconRed()) && !(!isRedTeam && robot().isBeaconBlue()))
            while(robot().driveAtAngleWhile(Robot.Side.RIGHT.angle, .5, ! robot().isCenterAtLine()))
                statusCheck();

        // hit button
        while(robot().driveForwardWhile(.5, ! robot().isBeaconTouching())) statusCheck();
    }
}

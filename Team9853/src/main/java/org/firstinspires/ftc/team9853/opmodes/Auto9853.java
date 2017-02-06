package org.firstinspires.ftc.team9853.opmodes;

import org.chathamrobotics.ftcutils.StoppedException;
import org.chathamrobotics.ftcutils.opmodes.templates.AutonomousOpMode;
import org.firstinspires.ftc.team9853.Robot9853;

/**
 * team specific template
 */

public abstract class Auto9853 extends AutonomousOpMode {
//    COMPONENTS    //
    private Robot9853 robot;

//    INSTANCE      //
    protected boolean isRedTeam;

//    CONSTRUCTOR   //

    /**
     * setup OpMode
     * @param isRedTeam whether or not this OpMode is for the red team.
     */
    public Auto9853(boolean isRedTeam) {this.isRedTeam = isRedTeam;}

//    METHODS       //

    /**
     * get robot
     * @return  the robot object
     */
    @Override
    public Robot9853 robot() {
        if(robot == null) this.robot = new Robot9853(hardwareMap, telemetry);

        return this.robot;
    }

    /**
     * hit the beacon
     * @throws StoppedException
     */
    protected void hitBeacon() throws StoppedException{
        while(robot().driveForwardWhile(.5, ! robot().isBeaconTouching())) statusCheck();
    }

    /**
     * check if the current beacon is the desired color
     * @return  whether the team and beacon color match
     */
    protected boolean isBeaconCaptured() {
        return robot().isBeaconRed() == isRedTeam;
    }
}

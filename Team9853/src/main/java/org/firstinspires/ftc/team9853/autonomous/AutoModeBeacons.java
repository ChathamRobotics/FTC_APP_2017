package org.firstinspires.ftc.team9853.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.util.Range;

import org.chathamrobotics.ftcutils.AutonomousOpMode;
import org.chathamrobotics.ftcutils.MRColorSensorV2;
import org.chathamrobotics.ftcutils.StoppedException;
import org.firstinspires.ftc.team9853.Robot9853;
import org.firstinspires.ftc.team9853.opmodes.Auto;

/**
 * beacon pressing autonomous
 */

public class AutoModeBeacons extends Auto {
//    CONSTANTS     //
    private static final long shootTime = 1000;
    private static final long reloadTime = 1000;

//    HARDWARE      //
    private MRColorSensorV2 lineSensor;

//    CONSTRUCTORS  //

    /**
     * Setup OpMode
     * @param isRedTeam   Whether the current team is red
     */
    public AutoModeBeacons(boolean isRedTeam) {
        this.isRedTeam = isRedTeam;
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
        while (! robot.isLeftAtLine()) {
            statusCheck();
            robot.driveAtAngle(Math.atan2(6/12d, this.isRedTeam ? 5/12d : -5/12d), .5);
        }
    }
}

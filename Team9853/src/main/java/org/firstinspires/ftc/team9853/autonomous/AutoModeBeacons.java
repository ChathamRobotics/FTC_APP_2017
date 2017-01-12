package org.firstinspires.ftc.team9853.autonomous;

import org.chathamrobotics.ftcutils.AutonomousOpMode;
import org.chathamrobotics.ftcutils.MRColorSensorV2;
import org.chathamrobotics.ftcutils.Robot;
import org.chathamrobotics.ftcutils.StoppedException;
import org.firstinspires.ftc.team9853.Robot9853;

/**
 * beacon pressing autonomous
 */

public class AutoModeBeacons extends AutonomousOpMode {
//    CONSTANTS     //
    private static final long shootTime = 1000;
    private static final long reloadTime = 1000;

//    COMPONENTS    //
    public Robot9853 robot;

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
     * Builds the robot
     * @return the robot
     */
    @Override
    public Robot buildRobot() {
        return new Robot9853(hardwareMap, telemetry);
    }

    /**
     * Initializes the robot
     */
    @Override
    public void initRobot() {
        super.initRobot();

        lineSensor = new MRColorSensorV2(hardwareMap.i2cDevice.get("LineSensor"), MRColorSensorV2.DEFAULT_I2C_ADDRESS);
        lineSensor.enableLed(true);
    }

    /**
     * called on start
     */
    public void runRobot() throws StoppedException, InterruptedException {
        for(long endTime = System.currentTimeMillis() + shootTime; System.currentTimeMillis() < endTime;) {
            statusCheck();
            robot.shooter.setPower(-.7);
        }

        for(long endTime = System.currentTimeMillis() + reloadTime; System.currentTimeMillis() < endTime;) {
            statusCheck();
            robot.belt.setPower(.7);
        }

        for(long endTime = System.currentTimeMillis() + shootTime; System.currentTimeMillis() < endTime;) {
            statusCheck();
            robot.shooter.setPower(-.7);
        }

        // Drive to beacon
        while (! lineSensor.isWhite()) {
            statusCheck();
            robot.driveAtPoint(isRedTeam ? -5/12d : 5/12d, 6/12d, .5);
        }
    }
}

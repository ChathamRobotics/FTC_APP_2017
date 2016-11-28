package org.firstinspires.ftc.team9853;

import org.chathamrobotics.ftcutils.AutonomousOpMode;
import org.chathamrobotics.ftcutils.AutonomousVisionOpMode;
import org.chathamrobotics.ftcutils.MRColorSensorV2;
import org.chathamrobotics.ftcutils.StoppedException;

import static org.lasarobotics.vision.opmode.VisionOpMode.beacon;

/**
 * beacon pressing autonomous
 */

public class AutoModeBeacons extends AutonomousVisionOpMode {
    // State
    private MRColorSensorV2 lineSensor;

    /*
     * Setup OpMode
     * @param {boolean} isRedTeam   Whether the current team is red
     */
    public AutoModeBeacons(boolean isRedTeam) {
        this.isRedTeam = isRedTeam;
    }

    /*
     * Initializes the robot
     */
    @Override
    public void initRobot() {
        super.initRobot();

        lineSensor = new MRColorSensorV2(hardwareMap.i2cDevice.get("LineSensor"), MRColorSensorV2.DEFAULT_I2C_ADDRESS);
        lineSensor.enableLed(true);
    }

    /*
     * called on start
     */
    public void runRobot() throws StoppedException, InterruptedException {
        // Drive to beacon
        while (! lineSensor.isWhite()) {
            statusCheck();
            driver.drive(isRedTeam ? -5/12d : 5/12d, 6/12d, 0, .5, false);
        }

        // drive forward until the beacon is in view
        while (beacon.getAnalysis().getConfidence() < 80) {
            statusCheck();
            driver.move(Math.PI/2, 0, .3);

            waitOneFullHardwareCycle();
        }
    }
}

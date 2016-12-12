package org.firstinspires.ftc.team9853;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.chathamrobotics.ftcutils.AutonomousOpMode;
import org.chathamrobotics.ftcutils.AutonomousVisionOpMode;
import org.chathamrobotics.ftcutils.MRColorSensorV2;
import org.chathamrobotics.ftcutils.StoppedException;

import static org.lasarobotics.vision.opmode.VisionOpMode.beacon;

/**
 * beacon pressing autonomous
 */

public class AutoModeBeacons extends AutonomousOpMode {
    // State
    private MRColorSensorV2 lineSensor;
    private DcMotor shooter, belt;

    long shootTime = 1000;
    long reloadTime = 1000;

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

        belt = hardwareMap.dcMotor.get("Belt");
        shooter = hardwareMap.dcMotor.get("Shooter");

        lineSensor = new MRColorSensorV2(hardwareMap.i2cDevice.get("LineSensor"), MRColorSensorV2.DEFAULT_I2C_ADDRESS);
        lineSensor.enableLed(true);
    }

    /*
     * called on start
     */
    public void runRobot() throws StoppedException, InterruptedException {
        for(long endTime = System.currentTimeMillis() + shootTime; System.currentTimeMillis() < endTime;) {
            statusCheck();
            shooter.setPower(-.7);
        }

        for(long endTime = System.currentTimeMillis() + reloadTime; System.currentTimeMillis() < endTime;) {
            statusCheck();
            belt.setPower(.7);
        }

        for(long endTime = System.currentTimeMillis() + shootTime; System.currentTimeMillis() < endTime;) {
            statusCheck();
            shooter.setPower(-.7);
        }

        // Drive to beacon
        while (! lineSensor.isWhite()) {
            statusCheck();
            driver.drive(isRedTeam ? -5/12d : 5/12d, 6/12d, 0, .5, false);
        }
    }
}

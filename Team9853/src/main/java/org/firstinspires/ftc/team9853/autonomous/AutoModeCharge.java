package org.firstinspires.ftc.team9853.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.chathamrobotics.ftcutils.AutonomousOpMode;
import org.chathamrobotics.ftcutils.OmniWheelDriver;
import org.chathamrobotics.ftcutils.StoppedException;


/**
 * charger autonomous - hits cap ball and shoots
 */
public class AutoModeCharge extends AutonomousOpMode {
//    CONSTANTS     //
    private static final long waitTime = 10000;
    private static final long driveTime = 2500;
    private static final long shootTime = 500;
    private static final long reloadTime = 2500;

//    HARDWARE      //
    private DcMotor sweeper, belt, shooter;

//    CONSTRUCTORS  //
    /*
     * Setup OpMode
     * @param {boolean} isRedTeam   Whether the current team is red
     */
    public AutoModeCharge(boolean isRedTeam) {
        this.isRedTeam = isRedTeam;
    }

//    METHODS       //
    /**
     * initializes the robot
     */
    @Override
    public void initRobot() {
        super.initRobot();

        sweeper = hardwareMap.dcMotor.get("Sweeper");
        belt = hardwareMap.dcMotor.get("Belt");
        shooter = hardwareMap.dcMotor.get("Shooter");
    }

    /**
     * called on start
     */
    public void runRobot() throws StoppedException {
        driver.offsetAngle = OmniWheelDriver.BACK_OFFSET;

        // Waits a little bit before starting autonomous
        waitFor(waitTime);

        // Drives to the shooting point
        for(long endTime = System.currentTimeMillis() + (3 * driveTime / 4); System.currentTimeMillis() < endTime;) {
            statusCheck();
            driver.move(Math.toRadians(90), 0, .7);
        }
        driver.move(0, 0, 0);

        // Shoots twice
        for(long endTime = System.currentTimeMillis() + shootTime; System.currentTimeMillis() < endTime;) {
            statusCheck();
            shooter.setPower(-.7);
        }
        shooter.setPower(0);

        for(long endTime = System.currentTimeMillis() + reloadTime; System.currentTimeMillis() < endTime;) {
            statusCheck();
            belt.setPower(1);
            sweeper.setPower(11);
        }
        belt.setPower(0);
        sweeper.setPower(0);

        for(long endTime = System.currentTimeMillis() + shootTime; System.currentTimeMillis() < endTime;) {
            statusCheck();
            shooter.setPower(-.7);
        }
        shooter.setPower(0);

        // Drives to center
        for(long endTime = System.currentTimeMillis() + (driveTime / 4); System.currentTimeMillis() < endTime;) {
            statusCheck();
            driver.move(Math.toRadians(90), 0, .7);
        }
        driver.move(0, 0, 0);
    }
}

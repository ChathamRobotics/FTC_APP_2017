package org.firstinspires.ftc.team9853.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.chathamrobotics.ftcutils.AutonomousOpMode;
import org.chathamrobotics.ftcutils.OmniWheelDriver;
import org.chathamrobotics.ftcutils.Robot;
import org.chathamrobotics.ftcutils.StoppedException;
import org.firstinspires.ftc.team9853.Robot9853;


/**
 * charger autonomous - hits cap ball and shoots
 */
public class AutoModeCharge extends AutonomousOpMode {
//    CONSTANTS     //
    private static final long waitTime = 10000;
    private static final long driveTime = 2500;
    private static final long shootTime = 500;
    private static final long reloadTime = 2500;

//    COMPONENTS    //
    public Robot9853 robot;

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
     * Builds team specific robot
     * @return  the robot
     */
    @Override
    public Robot buildRobot() {
        return new Robot9853(hardwareMap, telemetry);
    }

    /**
     * called on start
     */
    public void runRobot() throws StoppedException {
        robot.changeFront(Robot9853.Side.BACK);

        // Waits a little bit before starting autonomous
        waitFor(waitTime);

        // Drives to the shooting point
        for(long endTime = System.currentTimeMillis() + (3 * driveTime / 4); System.currentTimeMillis() < endTime;) {
            statusCheck();
            robot.driveForward(.7);
        }
        robot.stopDriving();

        // Shoots twice
        for(long endTime = System.currentTimeMillis() + shootTime; System.currentTimeMillis() < endTime;) {
            statusCheck();
            robot.shooter.setPower(-.7);
        }
        robot.shooter.setPower(0);

        for(long endTime = System.currentTimeMillis() + reloadTime; System.currentTimeMillis() < endTime;) {
            statusCheck();
            robot.belt.setPower(1);
            robot.sweeper.setPower(11);
        }
        robot.belt.setPower(0);
        robot.sweeper.setPower(0);

        for(long endTime = System.currentTimeMillis() + shootTime; System.currentTimeMillis() < endTime;) {
            statusCheck();
            robot.shooter.setPower(-.7);
        }
        robot.shooter.setPower(0);

        // Drives to center
        for(long endTime = System.currentTimeMillis() + (driveTime / 4); System.currentTimeMillis() < endTime;) {
            statusCheck();
            robot.driveForward(.7);
        }
        robot.stopDriving();
    }
}

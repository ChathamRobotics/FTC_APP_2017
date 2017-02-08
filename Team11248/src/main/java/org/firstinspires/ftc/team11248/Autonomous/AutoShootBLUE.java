package org.firstinspires.ftc.team11248.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.team11248.Robot11248;

/**
 * Team 11248 Shooter Autonomous
 */
@Autonomous(name = "AutoShootBLUE")
public class AutoShootBLUE extends LinearOpMode{

    /**
     * The robot being controlled.
     */
    private Robot11248 robot;

    public static double LIFT_UP = 0;
    public static double LIFT_DOWN = 1;

    //Time spent driving forward in milliseconds
    //private long timeDriving = 3000;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot11248(hardwareMap, telemetry);
        robot.init(); //Sets servos to right position.

        waitForStart();

        while (opModeIsActive()) {

           sleep(10000);

            robot.driveold(0,.8,0);
            sleep(750);

            //drive(0,0);
            robot.stop();
            sleep(500);

            robot.openCollector();
            robot.setShooter(.65f);
            sleep(750);

            robot.setConveyor(.2f);
            sleep(1150);
            robot.setConveyor(.8f);
            sleep(850);

            robot.conveyorOff();
            robot.shooterOff();
            robot.closeCollector();

            robot.driveold(0,1,0);
            sleep(1750);
            robot.stop();
            idle();
            break;
        }

    }

}

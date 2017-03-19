package org.firstinspires.ftc.team11248.Autonomous.Shoot;

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
@Autonomous(name = "ShootParkVortex")
public class ShootParkVortex extends LinearOpMode{

    /**
     * The robot being controlled.
     */
    private Robot11248 robot;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot11248(hardwareMap, telemetry);
        robot.init(); //Sets servos to right position.

        robot.deactivateServos();
        waitForStart(); //STAYS HERE UNTIL PLAY BUTTON
        robot.activateServos();
        robot.calibrateGyro();

        while (opModeIsActive()) {

           sleep(20000);

            robot.driveold(0,.8,0);
            sleep(1100);
            robot.stop();

            robot.openCollector();
            robot.setShooter(Robot11248.AUTO_SHOOTER_SPEED);
            sleep(1500);

            robot.setConveyor(.6f);
            sleep(500);
            robot.setConveyor(0);
            sleep(1000);
            robot.setConveyor(1);
            sleep(1000);

            robot.conveyorOff();
            robot.shooterOff();
            robot.closeCollector();

            robot.driveWithGyro(0,1,0);
            sleep(1750);
            robot.stop();
            idle();
            break;
        }

    }

}

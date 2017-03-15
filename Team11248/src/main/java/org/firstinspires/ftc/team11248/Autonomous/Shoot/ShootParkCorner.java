package org.firstinspires.ftc.team11248.Autonomous.Shoot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team11248.Robot11248;

/**
 * Team 11248 Shooter Autonomous
 */
@Autonomous(name = "ShootParkCorner")
public class ShootParkCorner extends LinearOpMode{

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

            //drive(0,0);
            robot.stop();
            sleep(500);

            robot.openCollector();
            robot.setShooter(.5f);
            sleep(750);

            robot.setConveyor(.2f);
            sleep(1150);
            robot.setConveyor(.7f);
            sleep(850);

            robot.conveyorOff();
            robot.shooterOff();
            robot.closeCollector();

            robot.driveWithGyro(1,0,0);
            sleep(2000);
            robot.stop();
            idle();
            break;
        }

    }

}

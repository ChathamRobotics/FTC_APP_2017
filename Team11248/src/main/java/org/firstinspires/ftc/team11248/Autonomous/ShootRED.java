package org.firstinspires.ftc.team11248.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team11248.Robot11248;

/**
 * Team 11248 Shooter Autonomous
 */
@Autonomous(name = "ShootRED")
public class ShootRED extends LinearOpMode{

    /**
     * The robot being controlled.
     */
    private Robot11248 robot;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot11248(hardwareMap, telemetry);
        robot.init(); //Sets servos to right position.

        waitForStart();

        while (opModeIsActive()) {

            sleep(6000);

            robot.driveold(0,.8,0);
            sleep(1000);

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

            robot.driveold(1,0,-.3);
            sleep(2500);
            robot.stop();

            sleep(10000);
            robot.driveold(0,1,0);
            sleep(2500);
            robot.stop();

            idle();
            break;
        }

    }

}

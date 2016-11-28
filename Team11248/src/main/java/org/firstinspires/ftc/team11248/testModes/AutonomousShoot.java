package org.firstinspires.ftc.team11248.testModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team11248.Robot11248;

/**
 * Team 11248 Shooter Autonomous
 */
@Autonomous(name = "AutonomousShoot", group = "General")
@Disabled
public class AutonomousShoot extends LinearOpMode{

    /**
     * The robot being controlled.
     */
    private Robot11248 robot;

    //Time spent driving forward in milliseconds
    private long timeDriving = 3000;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(); //Sets servos to right position.

        waitForStart();

        while (opModeIsActive()) {

            sleep(10000);

            robot.drive(.5,0,0,false);
            sleep(2000);

            robot.drive(0,0,0,false);
            sleep(500);

            robot.shooterOn();
            sleep(2000);

           // robot.movePaddleDown();
            sleep(3000);

           // robot.movePaddleUp();
            sleep(5000);

            robot.shooterOff();
            robot.drive(.5,0,0,false);
            sleep(5000);

            robot.drive(0,0,0,false);

            break;

        }
    }
}

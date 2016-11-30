package org.firstinspires.ftc.team11248;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Team 11248 Shooter Autonomous
 */
@Autonomous(name = "BeaconTest", group = "General")
public class BeaconTest extends LinearOpMode{

    /**
     * The robot being controlled.
     */
    private Robot11248BEACONCOPY robot;

    //Time spent driving forward in milliseconds
    private long timeDriving = 3000;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(); //Sets servos to right position.

        waitForStart();

        while (opModeIsActive()) {

            //Drive diagonal until hit white line
            while(!robot.getColor(1).isWhite())
                robot.drive(.5,.5,0,true);

            //Stop driving once white line hit.
            robot.drive(0,0,0,false);

            //Rotate counterclockwise until second color sensor is white.
            while(!robot.getColor(2).isWhite())
                robot.drive(0,0,.5,true);

            //Stop driving once white line hit.
            robot.drive(0,0,0,false);

            //If not red (blue) hit it, else move left and hit
            if(!robot.getColor(3).isRed()) {
                //HIT THING
            }
            else {
                robot.drive(.5,0,0,false);
                wait(500);
                robot.drive(0,0,0,false);
            }
            break;

        }
    }
}

package org.firstinspires.ftc.team11248;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.chathamrobotics.ftcutils.MRColorSensorV2;

/**
 * Team 11248 Shooter Autonomous
 */
@Autonomous(name = "BeaconTest", group = "General")
public class BeaconTest extends LinearOpMode{

    /**
     * The robot being controlled.
     */
    private Robot11248 robot;

    //Time spent driving forward in milliseconds
    private long timeDriving = 3000;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor[] motors = new DcMotor[8];
        Servo[] servos = new Servo[1];
        MRColorSensorV2[] colors = new MRColorSensorV2[3];
        for(int i = 0; i < motors.length; i++)
            motors[i] = hardwareMap.dcMotor.get(Robot11248.MOTOR_LIST[i]);
        for(int i = 0; i < servos.length; i++)
            servos[i] = hardwareMap.servo.get(Robot11248.SERVO_LIST[i]);
//        for(int i = 0; i < colors.length; i++)
//            colors[i] = new MRColorSensorV2(hardwareMap.i2cDevice.get(Robot11248.COLOR_LIST[i]));
        colors[2] = new MRColorSensorV2(hardwareMap.i2cDevice.get(Robot11248.COLOR_LIST[2]));
        robot = new Robot11248(motors,servos,colors,telemetry);
        robot.init(); //Sets servos to right position.

        waitForStart();

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
    }
}

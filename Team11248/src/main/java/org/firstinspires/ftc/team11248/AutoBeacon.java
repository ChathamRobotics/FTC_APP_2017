package org.firstinspires.ftc.team11248;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by tonytesoriero on 10/29/16.
 */
@Autonomous(name = "AutoB")

public class AutoBeacon extends LinearOpMode {

    public Robot11248 robot;

    @Override
    public void runOpMode() throws InterruptedException {

        //Initializes all sensors and motors
        DcMotor[] motors = new DcMotor[8];
        Servo[] servos = new Servo[1];
        for(int i = 0; i < motors.length; i++)
            motors[i] = hardwareMap.dcMotor.get(Robot11248.MOTOR_LIST[i]);
        for(int i = 0; i < servos.length; i++)
            servos[i] = hardwareMap.servo.get(Robot11248.SERVO_LIST[i]);
        robot = new Robot11248(motors,servos,telemetry);

        robot.init();

        waitForStart();

        while (opModeIsActive()) {

            sleep(10000); //wait idk why

            //move forward shoot
            robot.
            sleep(3500);
            break;

        }
    }

    public void drive(double x, double y, double rotat){

        //## CALCULATE VALUES ##

        // Takes regular x,y coordinates and converts them into polar (angle radius) cooridnates
        // Then turns angle by 90 degrees (Pi/4) to accommodate omni wheel axis

        // if x is 0, atan comes out undefined instead of PI/2 or 3PI/2
        if (x != 0) {
            angle = Math.atan(y / x);

        }else if(y > 0){//if it's 90 degrees use PI/2
            angle = Math.PI/2;

        }else{
            angle = (3 * Math.PI)/2;
        }

        r = Math.sqrt( (x*x) + (y*y) ) ;//get the radius (hypotenuse)
        //angle += (Math.PI/4);//take our angle and shift it 90 deg (PI/4)

        // BUG FIX atan() assumes x is always positive and angle in standard position
        // add PI to go to quadrant 2 or 3
        if(x<0){
            angle += Math.PI;
        }

        if(gamepad1.dpad_up){DP_angle = 0;}
        if(gamepad1.dpad_left){DP_angle = Math.PI/2;}
        if(gamepad1.dpad_down){DP_angle = Math.PI;}
        if(gamepad1.dpad_right){DP_angle = (3*Math.PI)/2;}


        FL = BR =  Math.sin(angle + DP_angle) * MAX_SPEED * r; //takes new angle and radius and converts them into the motor values
        FR = BL = Math.cos(angle + DP_angle) * MAX_SPEED * r;

        FL -= rotat; // implements rotation
        FR -= rotat;
        BL += rotat;
        BR += rotat;

        if(FL<=1 & FR<=1 & BR <=1 & BL<=1) {// Prevent fatal error
            frontLeft.setPower(FL); // -rot fl br y
            frontRight.setPower(FR); // -
            backLeft.setPower(-BL); // +
            backRight.setPower(-BR); //+
        }
    }
}

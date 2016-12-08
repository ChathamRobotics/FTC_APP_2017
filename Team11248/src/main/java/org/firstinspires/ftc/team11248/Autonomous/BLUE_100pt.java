package org.firstinspires.ftc.team11248.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team11248.Robot11248;

/**
 * blue autonomous 100 POINTS
 */
@TeleOp(name = "100ptBLUE")
public class BLUE_100pt extends LinearOpMode {

    Robot11248 robot;
    int A_SHOOT_TO_BEACON = 60;
    double rotationRatio = .004 ;
@Override
    public void runOpMode() throws InterruptedException {

        //STAYS HERE UNTIL INIT BUTTON

        //Initializes all sensors and motors
        DcMotor[] motors = new DcMotor[8];
        Servo[] servos = new Servo[2];
        I2cDevice[] color = new I2cDevice[2];
        GyroSensor gyro = hardwareMap.gyroSensor.get("gyro");

        for (int i = 0; i < motors.length; i++)
            motors[i] = hardwareMap.dcMotor.get(Robot11248.MOTOR_LIST[i]);
        for (int i = 0; i < servos.length; i++)
            servos[i] = hardwareMap.servo.get(Robot11248.SERVO_LIST[i]);
        for (int i = 0; i < color.length; i++)
            color[i] = hardwareMap.i2cDevice.get(Robot11248.COLOR_LIST[i]);

        robot = new Robot11248(motors, servos, color, gyro, telemetry);
        robot.init(); //Sets servos to right position.

        robot.activateColorSensors();
        robot.calibrateGyro(); //SETS ANGLE TOO 0 (BEFORE ANY MOVEMENT)

        waitForStart(); //STAYS HERE UNTIL PLAY BUTTON

        int state = 0;

        while (opModeIsActive() && !isStopRequested()) {
            //BEGIN AUTONOMOUS
            shootBallsStart(); //MOVES FORWARD AND SHOOT BALLS
            sleep(2000); //WAITS 2 SECONDS

            switch (state) {
                case 0: //DOES THIS UNTIL ANGLE GETS TO A_SHOOT_BEACON (60 deg)
                    robot.driveWithGyro2(0, 0, A_SHOOT_TO_BEACON); //ROTATE TO A_SHOOT_TO_BEACON
                    if(Robot11248.angleWithinThreshold(robot.getGyroAngle(),A_SHOOT_TO_BEACON)) { //WHEN ANGLE REACHED
                        state++; //NEXT STATE
                        robot.stop(); //STOP MOVING
                    }
                    break;
                case 1: //DOES THIS UNTIL IT REACHES A LINE
                    robot.driveold(.25, 0, 0,false); //DRIVE LEFT .25
                    if(robot.hitLine()) { //WHEN WHITE LINE FOUND
                        state++; //NEXT STATE
                        robot.stop(); //STOP MOVING
                    }
                    break;
                case 2: //DOES THIS UNTIL ANGLE GETS TO 90
                    robot.driveWithGyro2(0, 0, 90); //ROTATE TO 90 deg
                    if(Robot11248.angleWithinThreshold(robot.getGyroAngle(),90)) {
                        state++; //NEXT STATE
                        robot.driveold(0, -.5, 0, false); //MOVE BACKWARD -.5
                        sleep(1000); //WAIT A SECOND
                        robot.stop(); //STOP MOVING
                    }
                    break;
                case 3: //
                    robot.driveold(.3, 0, 0, false);
                    if (robot.isBeaconBlue() || robot.isBeaconRed()) { //IF THE BEACON IS RED OR BLUE
                        state++; //NEXT STATE
                        robot.stop(); //STOP MOVING
                        if (robot.isBeaconBlue()) { //WHEN BEACON IS BLUE
                            robot.moveBeaconOut(); //PUSH BEACON
                        }
                        else { //BEACON IS NOT BLUE (AKA ITS RED)
                            robot.driveold(0, .5, 0, false); //MOVE UP .5
                            sleep(500); //WAIT .5 SECONDS
                            robot.stop(); //STOP MOVING
                            robot.moveBeaconOut(); //PUSH BEACON
                        }
                    }
                case 4: //STOPS OP MODE
                    robot.stop();
                    idle();
                    break;
                default:
                    //Nothing (Here for testing specific sections of switch)
                    break;
            }
        }
   }

    public void shootBallsStart(){
        robot.driveold(0,.8,0,false);
        sleep(1200);

        //drive(0,0);
        robot.stop();
        sleep(500);

        robot.shooterOn();
        sleep(750);

        robot.conveyorOn();
        sleep(2500);

        robot.conveyorOff();
        robot.shooterOff();
    }

//    public void driveWithGyro(double x, double y, int targetAngle){
//
//        int currentAngle = robot.getGyroAngle();
//        int net = currentAngle - targetAngle;
//        double rotation = -.25;
//
//        if(net > 180) { // if passes 0
//            if(currentAngle > 180) //counterclockwise past 0
//                net = (360 - currentAngle) + targetAngle;
//
//            else
//                net = (targetAngle - 360) + currentAngle;
//        }
//
//        //rotation = -.25;//net * rotationRatio + .25;
//
//        telemetry.addData("1:", "Heading: " + robot.getGyroAngle());
//        telemetry.addData("2:", "Net: " + net);
//        telemetry.addData("3: ", "Speed: " +rotation);
//        telemetry.addData("4: ",  "Target: " + targetAngle);
//
//        telemetry.update();
//
//        robot.driveold(x,y,rotation,false);
//    }
}

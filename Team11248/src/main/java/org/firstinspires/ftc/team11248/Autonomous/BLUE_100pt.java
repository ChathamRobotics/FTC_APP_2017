package org.firstinspires.ftc.team11248.Autonomous;

import android.os.PowerManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team11248.Robot11248;

/**
 * blue autonomous 100 POINTS
 */
@Autonomous(name = "100ptBLUE")
public class BLUE_100pt extends LinearOpMode {

    Robot11248 robot;

    final int STOP_DELAY = 500;
    int A_SHOOT_TO_BEACON = 38;
    double rotationRatio = .004 ;

    int TIME_TO_FIRST_COLOR = 250;
    int TIME_TO_OTHER_COLOR = 500;
    int TIME_FORWARD_TO_BEACON = 550;
@Override
    public void runOpMode() throws InterruptedException {

        //STAYS HERE UNTIL INIT BUTTON

        //Initializes all sensors and motors
        DcMotor[] motors = new DcMotor[8];
        Servo[] servos = new Servo[4];
        I2cDevice color = hardwareMap.i2cDevice.get(Robot11248.COLOR);
        GyroSensor gyro = hardwareMap.gyroSensor.get(Robot11248.GYRO);
        OpticalDistanceSensor line = hardwareMap.opticalDistanceSensor.get(Robot11248.LINE);
        UltrasonicSensor sonar = hardwareMap.ultrasonicSensor.get(Robot11248.SONAR);

        for (int i = 0; i < motors.length; i++)
            motors[i] = hardwareMap.dcMotor.get(Robot11248.MOTOR_LIST[i]);
        for (int i = 0; i < servos.length; i++)
            servos[i] = hardwareMap.servo.get(Robot11248.SERVO_LIST[i]);

        robot = new Robot11248(motors, servos, color, gyro, line, sonar,telemetry);
        robot.init(); //Sets servos to right position.

        robot.activateColorSensors();
        robot.calibrateGyro(); //SETS ANGLE TOO 0 (BEFORE ANY MOVEMENT)

        waitForStart(); //STAYS HERE UNTIL PLAY BUTTON

        int state = -1;

        while (opModeIsActive() && !isStopRequested()) {
            //BEGIN AUTONOMOUS


            switch (state) {
                case -1://6 seconds
                    shootBallsStart(); //MOVES FORWARD AND SHOOT BALLS
                    sleep(1000);
                    state++;
                    break;

                case 0:
                    robot.driveold(.5, .4, 0); //DRIVE DIAGONAL
                    if(robot.hitLine()) { //WHEN WHITE LINE FOUND
                        robot.stop(); //STOP MOVING
                        state++; //NEXT STATE
                    }
                    break;

                case 1:
                    robot.stop();
                    sleep(STOP_DELAY);

                    retrieveBeacon(1300, .25, 100);
                    state++;
                    break;

                case 2: //DOES THIS UNTIL IT REACHES A LINE
                    driveAgainstWall(1, 0, 5); //MOVE FORWARD
                    if(robot.hitLine()) { //WHEN WHITE LINE FOUND
                        state++; //NEXT STATE
                        robot.stop(); //STOP MOVING
                    }
                    break;

                //SECOND BEACON
                case 3:
                    robot.stop();
                    sleep(STOP_DELAY);

                    retrieveBeacon(1300, .25, 100);
                    break;


                default:
                    robot.stop();
                    idle();
                    break;
            }
        }
   }

    public void shootBallsStart(){
        robot.driveold(0,.8,0);
        sleep(1200);

        //drive(0,0);
        robot.stop();
        sleep(500);

        robot.openCollector();
        robot.shooterOn();
        sleep(1000);

        robot.setConveyor(.2f);
        sleep(2500);

        robot.conveyorOff();
        robot.shooterOff();
        robot.closeCollector();

        //4.95 seconds
    }


    public void retrieveBeacon(long x, double speed, int distance){

        //Y ADJUSTMENT
        robot.driveWithGyro(speed, 0, 0);
        if(robot.getDistanceCM() < distance)
            robot.stop();
        else
            return;


        //X ADJUSTMENT
        robot.driveWithGyro(0, -speed, 0);
        sleep(x);
        robot.stop();
        sleep(STOP_DELAY);


        if (robot.isBeaconBlue()) { //WHEN BEACON IS BLUE
            robot.moveBeaconOut(); //PUSH BEACON
            sleep(1000);
            robot.moveBeaconIn();
        }

        else if (robot.isBeaconRed()){ //BEACON IS NOT BLUE (AKA ITS RED)

            robot.driveWithGyro(0, -speed, 0); //MOVE UP .5
            sleep(TIME_TO_OTHER_COLOR); //WAIT .5 SECONDS
            robot.stop(); //STOP MOVING

            robot.moveBeaconOut();
            sleep(1000);//PUSH BEACON
            robot.moveBeaconIn();

        }else{}
    }

    public void driveAgainstWall(double speed, int angle, int distance){

        double netDist = robot.getDistanceCM() - distance;
        double y =0;

        if(netDist > 2)
            y = speed;
        else if (netDist < 2)
            y = -speed;

        robot.driveWithGyro(speed, y, angle);

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

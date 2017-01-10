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
    int state = -1;


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


        while (opModeIsActive() && !isStopRequested()) {
            //BEGIN AUTONOMOUS
            telemetry.update();
            telemetry.addData("isBlue", robot.isBeaconBlue());
            telemetry.addData("isRed", robot.isBeaconRed());
            telemetry.addData("sonar", robot.getSonarValue());

            switch (state) {
                case -1://6 seconds
                    shootBallsStart(); //MOVES FORWARD AND SHOOT BALLS
                    sleep(1000);
                    state++;
                    break;

                case 0:
                    robot.driveold(.3, .3, 0); //DRIVE DIAGONAL
                    if(robot.hitLine()) { //WHEN WHITE LINE FOUND
                        robot.stop(); //STOP MOVING
                        sleep(STOP_DELAY);
                        state++; //NEXT STATE
                    }
                    break;
                case 1:
                    if(robot.moveToAngle(0)) state++;
                    break;
                case 2:
                    //Y ADJUSTMENT
                    robot.driveold(.3, 0, 0);
                    if (robot.getSonarValue() < 12){
                        robot.stop();
                        sleep(STOP_DELAY);
                        state++;
                    }
                    break;
                case 3:
                    retrieveBeacon(1300, .4);
                    state++;
                    break;
//                case 4: //DOES THIS UNTIL IT REACHES A LINE
//                    driveAgainstWall(1, 0, 25); //MOVE FORWARD
//                    if(robot.hitLine()) { //WHEN WHITE LINE FOUND
//                        robot.stop(); //STOP MOVING
//                        sleep(STOP_DELAY);
//                        state++; //NEXT STATE
//                    }
//                    break;
//                //SECOND BEACON
//                case 5:
//                    retrieveBeacon(1300, .25);
//                    state++;
//                    break;
                default:
                    robot.stop();
                    idle();
                    break;
            }
        }
   }

    public void shootBallsStart(){
        robot.driveWithGyro(0,.8,0);
        sleep(500);

        //drive(0,0);
        robot.stop();
        sleep(500);

        robot.openCollector();
        robot.shooterOn();
        sleep(750);

        robot.setConveyor(.2f);
        sleep(2500);

        robot.conveyorOff();
        robot.shooterOff();
        robot.closeCollector();

        //4.95 seconds
    }

    public void pushBeacon() {
        robot.moveBeaconOut(); //PUSH BEACON
        sleep(1000);
        robot.moveBeaconIn();
    }

    public void retrieveBeacon(long x, double speed) {
        //X ADJUSTMENT
        robot.driveold(0, -speed, 0);
        sleep(x);
        robot.stop();
        sleep(STOP_DELAY);

        if (robot.isBeaconBlue())//WHEN BEACON IS BLUE
            pushBeacon();
        else if (robot.isBeaconRed()) { //BEACON IS NOT BLUE (AKA ITS RED)
            robot.driveWithGyro(0, -speed, 0); //MOVE LEFT .5
            sleep(TIME_TO_OTHER_COLOR); //WAIT .5 SECONDS
            robot.stop(); //STOP MOVING

            pushBeacon();
        }
    }

    public void driveAgainstWall(double speed, int angle, int distance){
        int SONAR_THRESHOLD = 5;
        double netDist = robot.getSonarValue() - distance;
        double y =0;

        if(Math.abs(netDist)> SONAR_THRESHOLD) {
            y = Math.abs(netDist) * .003 + .25;
            if(netDist<0) y*=-1;
        }
        robot.driveWithGyro( y , speed, angle);
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

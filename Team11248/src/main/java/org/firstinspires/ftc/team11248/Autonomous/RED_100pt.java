package org.firstinspires.ftc.team11248.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team11248.Robot11248;

/**
 * blue autonomous 100 POINTS
 */
@Autonomous(name = "100ptRED")
public class RED_100pt extends LinearOpMode {

    Robot11248 robot;
    int A_SHOOT_TO_BEACON = 38;
    OpticalDistanceSensor lineSensor;
    private static final double OPTICAL_THRESHOLD_LOW = .9;
    private static final double OPTICAL_THRESHOLD_HIGH = 1;
    double rotationRatio = .004 ;

    int TIME_TO_OTHER_COLOR = 500;
    int TIME_TO_ROTATE = 2000;
    @Override
    public void runOpMode() throws InterruptedException {

        //STAYS HERE UNTIL INIT BUTTON

        //Initializes all sensors and motors
        DcMotor[] motors = new DcMotor[8];
        Servo[] servos = new Servo[2];
        I2cDevice[] color = new I2cDevice[2];
        GyroSensor gyro = hardwareMap.gyroSensor.get("gyro");
        lineSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");

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

        int state = -1;

        while (opModeIsActive() && !isStopRequested()) {
            //BEGIN AUTONOMOUS


            switch (state) {
                case -1://6 seconds
                    shootBallsStart(); //MOVES FORWARD AND SHOOT BALLS
                    sleep(1000); //WAITS 2 SECONDS
                    state++;
                case 0: //DOES THIS UNTIL ANGLE GETS TO A_SHOOT_BEACON (60 deg)
//                    robot.driveWithGyro2(0, 0, A_SHOOT_TO_BEACON); //ROTATE TO A_SHOOT_TO_BEACON
//                    if(Robot11248.angleWithinThreshold(robot.getGyroAngle(),A_SHOOT_TO_BEACON)) { //WHEN ANGLE REACHED
//                        state++; //NEXT STATE
//                        robot.stop(); //STOP MOVING
//                    }
                    robot.driveold(0,0,.5);
                    sleep(TIME_TO_ROTATE);
                    robot.driveold(-.5, .4, 0); //DRIVE DIAGONAL
                    if(hitLine()) { //WHEN WHITE LINE FOUND
                        state++; //NEXT STATE
                        robot.stop(); //STOP MOVING
                    }
                    state++;
                    break;
                case 1: //DOES THIS UNTIL IT REACHES A LINE

                    robot.driveold(0,.4,0);
                    if(robot.isBeaconRed()||robot.isBeaconBlue()){
                        sleep(1000);
                        robot.stop();
                        state++;
                    }
                    //sleep(1000);
                    //  robot.stop();

                    break;
                case 2: //DOES THIS UNTIL ANGLE GETS TO 90
//                    robot.driveWithGyro2(0, 0, 0); //ROTATE TO 90 deg
//                    if(Robot11248.angleWithinThreshold(robot.getGyroAngle(),0)) {
//                        state++; //NEXT STATE
//                       // robot.driveold(0, -.5, 0, false); //MOVE BACKWARD -.5
//                        sleep(1000); //WAIT A SECOND
//                        robot.stop(); //STOP MOVING
//                  }
                    retrieveBeacon(400,0,.4);
                    state++;
                    break;
                case 3:
                    robot.driveold(.8, 0, 0);
                    if(hitLine()) { //WHEN WHITE LINE FOUND
                        state++; //NEXT STATE
                        robot.stop(); //STOP MOVING
                    }
                    state++;
                    break;

                case 4: //STOPS OP MODE
//                    robot.driveold(0,.4,0);
//                    if(robot.isBeaconRed()||robot.isBeaconBlue()){
//                        robot.stop();
//                        state++;
//                    }
                    state++;
                    break;

                case 5:
                    retrieveBeacon(400,0,.4);
                    state++;
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

        robot.shooterOn();
        sleep(750);

        robot.setConveyor(.3f);
        sleep(2500);

        robot.conveyorOff();
        robot.shooterOff();

        //4.95 seconds
    }

    public boolean hitLine(){
        return (lineSensor.getLightDetected() < OPTICAL_THRESHOLD_HIGH &&
                lineSensor.getLightDetected() >= OPTICAL_THRESHOLD_LOW);
    }

    public void retrieveBeacon(long x, long y, double speed){

        //X ADJUSTMENT
        robot.driveold(0,speed,0);
        sleep(x);
        robot.stop();
        sleep(250);

        //Y ADJUSTMENT
        robot.driveold(speed,0,0);
        sleep(y);
        robot.stop();

        if (robot.isBeaconBlue()) { //WHEN BEACON IS BLUE
            robot.moveBeaconOut(); //PUSH BEACON
            sleep(1000);
            robot.moveBeaconIn();

        }
        else if (robot.isBeaconRed()){ //BEACON IS NOT BLUE (AKA ITS RED)
            robot.driveold(0, speed, 0); //MOVE UP .5
            sleep(TIME_TO_OTHER_COLOR); //WAIT .5 SECONDS
            robot.stop(); //STOP MOVING
            robot.moveBeaconOut();
            sleep(1000);//PUSH BEACON
            robot.moveBeaconIn();

        }else{

        }


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

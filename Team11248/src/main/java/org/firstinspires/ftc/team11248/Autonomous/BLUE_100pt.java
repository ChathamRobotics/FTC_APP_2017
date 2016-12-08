package org.firstinspires.ftc.team11248.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team11248.Robot11248;

/**
 * Created by Tony_Air on 12/7/16.
 */
@TeleOp(name = "100ptBLUE")
public class BLUE_100pt extends LinearOpMode {

    Robot11248 robot;
    int threshold = 2;
    int A_SHOOT_TO_BEACON = 60;
    double rotationRatio = .004 ;
@Override
    public void runOpMode() throws InterruptedException {

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
        robot.calibrateGyro();

        waitForStart();

        int state = 0;

        while (opModeIsActive() && !isStopRequested()) {
            shootBallsStart();
            sleep(2000);

            switch (state) {
                case 0:
                    driveWithGyro2(0, 0, A_SHOOT_TO_BEACON);
                    if (robot.getGyroAngle() >= A_SHOOT_TO_BEACON)
                        robot.stop();
                    idle();
                    break;
                case 1:
                    driveWithGyro(1, 0, A_SHOOT_TO_BEACON);
                    if (robot.hitLine()) {
                        state++;
                        robot.stop();
                    }
                    break;
                case 2:
                    driveWithGyro(0, 0, 90);
                    if (robot.getGyroAngle() >= 90) {
                        state++;
                        robot.driveold(0, -.5, 0, false);
                        sleep(1000);
                        robot.stop();
                    }
                    break;
                case 3:
                    robot.driveold(.3, 0, 0, false);
                    if (robot.isBeaconBlue() || robot.isBeaconRed()) {
                        state++;
                        robot.stop();
                        if (robot.isBeaconBlue()) {
                            robot.moveBeaconOut();
                        } else if (robot.isBeaconRed()) {
                            robot.driveold(0, .5, 0, false);
                            sleep(500);
                            robot.stop();
                            robot.moveBeaconOut();
                        }
                    }
                case 4:
                    shootBallsStart();
                    //STOP OP MODE HERE
                    robot.stop();
                    idle();
                    break;


                //OLD CODE
//            while(robot.getGyroAngle()<A_SHOOT_TO_BEACON) {
//                driveWithGyro(0, 0, A_SHOOT_TO_BEACON);
//            }
//
//            while(!robot.hitLine())   //Find Line
//                driveWithGyro(1,0,A_SHOOT_TO_BEACON);
//
//            robot.stop();
//
//            while(robot.getGyroAngle()< 90) // flat to wall
//                driveWithGyro(0,0,90);


//           //LEFT AND RI˝HT ADJUSTMENTS TO LINE UP WITH BEACON
//                robot.driveold(0,-.5, 0, false);
//                sleep(1000);
//                robot.stop();
//
//
//                //Beacon Time
//
//                while(!robot.isBeaconBlue() && !robot.isBeaconRed()) robot.driveold(.3, 0, 0, false);
//                robot.stop();
//
//                if(robot.isBeaconBlue()) {
//                    robot.moveBeaconOut();
//
//                }else if(robot.isBeaconRed()) {
//                    robot.driveold(0,.5,0,false);
//                    sleep(500);
//                    robot.stop();
//                    robot.moveBeaconOut();
//
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

    public void driveWithGyro(double x, double y, int targetAngle){

        int currentAngle = robot.getGyroAngle();
        int net = currentAngle - targetAngle;
        double rotation = -.25;

        if(net > 180) { // if passes 0
            if(currentAngle > 180) //counterclockwise past 0
                net = (360 - currentAngle) + targetAngle;

            else
                net = (targetAngle - 360) + currentAngle;
        }

        //rotation = -.25;//net * rotationRatio + .25;

        telemetry.addData("1:", "Heading: " + robot.getGyroAngle());
        telemetry.addData("2:", "Net: " + net);
        telemetry.addData("3: ", "Speed: " +rotation);
        telemetry.addData("4: ",  "Target: " + targetAngle);

        telemetry.update();

        robot.driveold(x,y,rotation,false);
    }

    public void driveWithGyro2(double x, double y, int targetAngle){
        int currentAngle = robot.getGyroAngle();

        double rotation = -.25;

        telemetry.addData("1:", "Heading: " + robot.getGyroAngle());
        telemetry.addData("3: ", "Speed: " +rotation);
        telemetry.addData("4: ",  "Target: " + targetAngle);

        telemetry.update();

        if(Math.abs(robot.getGyroAngle() - targetAngle) <= .01)
            robot.driveold(x,y,0,false);
        else
            robot.driveold(x,y,rotation,false);
    }
}

package org.firstinspires.ftc.team11248.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.team11248.Robot11248;



/**
 * Created by Tony_Air on 12/7/16.
 */

@TeleOp(name = "GyroDriveTest")
@Disabled
public class TestGyroBeacon extends OpMode {

   Robot11248 robot;


    int threshold = 2;
    boolean done;


    @Override
    public void init() {

        robot = new Robot11248(hardwareMap, telemetry);
        robot.init(); //Sets servos to right position.
        robot.activateColorSensors();
        robot.calibrateGyro();

    }

    @Override
    public void loop() {


       driveAgainstWall(.4,0,50);




        telemetry.addData("01: ", robot.getGyroAngle());
        telemetry.addData("02: ", 0);

    }

    public void driveWithGyro(double x, double y, double rotationSpeed, int angle){

        double rotation = 0;

        if(robot.getGyroAngle() > angle + threshold)
            rotation = rotationSpeed;

        else if (robot.getGyroAngle() < angle - threshold)
            rotation = -rotationSpeed;


        robot.driveold(x,y,rotation,false);
    }

    public void driveWithGyro2(double x, double y, int targetAngle){
        int currentAngle = robot.getGyroAngle();

        double rotation = -.3;

        telemetry.addData("1:", "Heading: " + robot.getGyroAngle());
        telemetry.addData("3: ", "Speed: " +rotation);
        telemetry.addData("4: ",  "Target: " + targetAngle);

        telemetry.update();

        if(Math.abs(robot.getGyroAngle() - targetAngle) <= 3) {
            robot.driveold(x, y, 0, false);
            done = true;
        }
        else {
            robot.driveold(x, y, rotation, false);
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
}

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
//@Disabled
public class TestGyroBeacon extends OpMode {

   Robot11248 robot;


    int threshold = 2;
    boolean done;


    @Override
    public void init() {
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

        robot = new Robot11248(motors, servos, color, gyro, line, sonar, telemetry);
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

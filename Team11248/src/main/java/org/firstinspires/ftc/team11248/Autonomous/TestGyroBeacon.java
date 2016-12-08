package org.firstinspires.ftc.team11248.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team11248.Robot11248;



/**
 * Created by Tony_Air on 12/7/16.
 */

@TeleOp(name = "GyroDriveTest")
public class TestGyroBeacon extends OpMode {

   Robot11248 robot;


    int threshold = 2;


    @Override
    public void init() {
        DcMotor[] motors = new DcMotor[8];
        Servo[] servos = new Servo[2];
        I2cDevice[] color = new I2cDevice[2];
        GyroSensor gyro = hardwareMap.gyroSensor.get("gyro");

        for(int i = 0; i < motors.length; i++)
            motors[i] = hardwareMap.dcMotor.get(Robot11248.MOTOR_LIST[i]);
        for(int i = 0; i < servos.length; i++)
            servos[i] = hardwareMap.servo.get(Robot11248.SERVO_LIST[i]);
        for(int i = 0; i < color.length; i++)
            color[i] = hardwareMap.i2cDevice.get(Robot11248.COLOR_LIST[i]);


        robot = new Robot11248(motors,servos, color, gyro, telemetry);
        robot.init(); //Sets servos to right position.
        robot.activateColorSensors();
        robot.calibrateGyro();

    }

    @Override
    public void loop() {


        while(!robot.isBeaconBlue()) {
           driveWithGyro(.5,0,.5,0);

        }




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
}

package org.firstinspires.ftc.team11248.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.team11248.Robot11248;

/**
 * Created by Tony_Air on 12/10/16.
 */

@Autonomous(name = "driveWithGyro")
public class BLUE_70pt extends OpMode {

    Robot11248 robot;
    @Override
    public void init() {
        //Initializes all sensors and motors
        DcMotor[] motors = new DcMotor[8];
        Servo[] servos = new Servo[2];
        I2cDevice[] color = new I2cDevice[2];
        GyroSensor gyro = hardwareMap.gyroSensor.get("gyro");
        //lineSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");

        for (int i = 0; i < motors.length; i++)
            motors[i] = hardwareMap.dcMotor.get(Robot11248.MOTOR_LIST[i]);
        for (int i = 0; i < servos.length; i++)
            servos[i] = hardwareMap.servo.get(Robot11248.SERVO_LIST[i]);
        for (int i = 0; i < color.length; i++)
            color[i] = hardwareMap.i2cDevice.get(Robot11248.COLOR_LIST[i]);

        robot = new Robot11248(motors, servos, color, gyro, telemetry);
        robot.init(); //Sets servos to right position.

       // robot.activateColorSensors();
        robot.calibrateGyro(); //SETS ANGLE TOO 0 (BEFORE ANY MOVEMENT)
    }

    @Override
    public void loop() {

        driveWithGyro(0,0,0);

    }

    public void driveWithGyro(double x, double y, int targetAngle){

        int currentAngle = robot.getGyroAngle();
        int net = currentAngle - targetAngle;
        double rotation = .3;

        if(net > 180) { // if passes 0
            if(currentAngle > 180) //counterclockwise past 0
                net = (currentAngle - 360) + targetAngle;

            else
                net = (360 - targetAngle) + currentAngle;
        }

        if(net<0){
            rotation *= -1;
        }

        //rotation = -.25;//net * rotationRatio + .25;

        telemetry.addData("1:", "Heading: " + robot.getGyroAngle());
        telemetry.addData("2:", "Net: " + net);
        telemetry.addData("3: ", "Speed: " +rotation);
        telemetry.addData("4: ",  "Target: " + targetAngle);

        telemetry.update();

        robot.driveold(x,y,rotation,false);
    }
}

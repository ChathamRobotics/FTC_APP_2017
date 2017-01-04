package org.firstinspires.ftc.team11248.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.team11248.Robot11248;

/**
 * Created by Tony_Air on 12/10/16.
 */

@Autonomous(name = "driveWithGyro")
public class BLUE_70pt extends OpMode {

    Robot11248 robot;
    VoltageSensor voltageSensor;
    @Override
    public void init() {
        //Initializes all sensors and motors
        DcMotor[] motors = new DcMotor[8];
        Servo[] servos = new Servo[4];
        I2cDevice color = hardwareMap.i2cDevice.get(Robot11248.COLOR);
        GyroSensor gyro = hardwareMap.gyroSensor.get(Robot11248.GYRO);
        OpticalDistanceSensor line = hardwareMap.opticalDistanceSensor.get(Robot11248.LINE);

        for (int i = 0; i < motors.length; i++)
            motors[i] = hardwareMap.dcMotor.get(Robot11248.MOTOR_LIST[i]);
        for (int i = 0; i < servos.length; i++)
            servos[i] = hardwareMap.servo.get(Robot11248.SERVO_LIST[i]);

        robot = new Robot11248(motors, servos, color, gyro, line, null, telemetry);
        robot.init(); //Sets servos to right position.
        //robot.activateColorSensors();
        robot.calibrateGyro(); //SETS ANGLE TOO 0 (BEFORE ANY MOVEMENT)


    }

    @Override
    public void loop() {

        robot.driveWithGyro(.3,0,0);

    }
}

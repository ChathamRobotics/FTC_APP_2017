package org.firstinspires.ftc.team11248;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cController.I2cPortReadyCallback;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.chathamrobotics.ftcutils.MRColorSensorV2;
import org.chathamrobotics.ftcutils.UltrasonicSensor_HCSR04;

import java.util.concurrent.locks.Lock;

@TeleOp(name = "SensorValTest")
public class SensorValTest extends OpMode {

    // I2C address, registers, and commands
    private final byte COLOR_SENSOR_GREEN_ADDR = 0x3A; //Green
    private final byte COLOR_SENSOR_YELLOW_ADDR = 0x3C; //Yellow
    private final byte COLOR_SENSOR_BEACON_ADDR = 0x3E; //Beacon

    public Robot11248 robot;



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

    // Respond to gamepad input.
    public void loop() {

        telemetry.addData("01: ", "Line Sensor: " + robot.getLineSensorValue());
        telemetry.addData("02: ", "hitLine: " + robot.hitLine());
        telemetry.addData("03: ", "ColorBeacon: " + robot.getColorBeacon());
        telemetry.addData("04: ", "isBlue: " + robot.isBeaconBlue());
        telemetry.addData("05: ", "isRed: " + robot.isBeaconRed());
        telemetry.addData("06: ", "Heading: " + robot.getGyroAngle());
        telemetry.addData("07: ", "UltrasonicRaw: " + robot.getSonarValue());
        telemetry.addData("08: ", "UltrasonicIN: " + robot.getDistanceIN());
        telemetry.addData("09: ", "UltrasonicCM: " + robot.getDistanceCM());
        telemetry.addData("10: ", "UltrasonicMM: " + robot.getDistanceMM());



    }

}
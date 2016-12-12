package org.firstinspires.ftc.team11248;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cController.I2cPortReadyCallback;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.chathamrobotics.ftcutils.MRColorSensorV2;

import java.util.concurrent.locks.Lock;

@TeleOp(name = "BeaconValTest")
public class BeaconValTest extends OpMode {

    // I2C address, registers, and commands
    private final byte COLOR_SENSOR_GREEN_ADDR = 0x3A; //Green
    private final byte COLOR_SENSOR_YELLOW_ADDR = 0x3C; //Yellow
    private final byte COLOR_SENSOR_BEACON_ADDR = 0x3E; //Beacon

    public Robot11248 robot;

   // OpticalDistanceSensor odsSensor;

    /* In init(), we get a handle on the color sensor itself and its controller.
     * We need access to the cycle of events on the sensor (when the port is
     * ready for read/write activity). The portIsReady() method is registered
     * as a callback.
     */
    public void init() {

        //Initializes all sensors and motors
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

        //gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");


     //   gyro.calibrate();

       // odsSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");
    }

    // Respond to gamepad input.
    public void loop() {

        telemetry.addData("02: ", "ColorYellow: " + robot.getColorYellow());
        telemetry.addData("03: ", "ColorBeacon: " + robot.getColorBeacon());
        telemetry.addData("04: ", "isBlue: " + robot.isBeaconBlue());
        telemetry.addData("05: ", "isRed: " + robot.isBeaconRed());

        telemetry.addData("06: ", "Heading: " + robot.getGyroAngle());
//        telemetry.addData("Int. Ang. %03d", angleZ);
//        telemetry.addData("X av. %03d", xVal);
//        telemetry.addData("Y av. %03d", yVal);
//        telemetry.addData("Z av. %03d", zVal);


    }

}
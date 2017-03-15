package org.firstinspires.ftc.team11248;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SensorValTest")
public class SensorValTest extends OpMode {

    // I2C address, registers, and commands
    private final byte COLOR_SENSOR_GREEN_ADDR = 0x3A; //Green
    private final byte COLOR_SENSOR_YELLOW_ADDR = 0x3C; //Yellow
    private final byte COLOR_SENSOR_BEACON_ADDR = 0x3E; //Beacon

    public Robot11248 robot;



    public void init() {

        robot = new Robot11248(hardwareMap, telemetry);
        robot.init(); //Sets servos to right position.
        robot.activateColorSensors();
        robot.calibrateGyro();

    }

    // Respond to gamepad input.
    public void loop() {

        telemetry.addData("01: ", "Line Sensor: " + robot.getLineSensorValue());
        telemetry.addData("02: ", "hitLine: " + robot.hitLine());
        telemetry.addData("03: ", "Line Sensor 2: " + robot.getLineSensor2Value());
        telemetry.addData("02: ", "hitLine: " + robot.hitLine2());
        telemetry.addData("04: ", "ColorBeacon: " + robot.getColorBeacon());
        telemetry.addData("05: ", "isBlue: " + robot.isBeaconBlue());
        telemetry.addData("06: ", "isRed: " + robot.isBeaconRed());
        telemetry.addData("07: ", "Heading: " + robot.getGyroAngle());
        telemetry.addData("08: ", "UltrasonicRaw: " + robot.getSonarValue());
//        telemetry.addData("08: ", "UltrasonicIN: " + robot.getDistanceIN());
//        telemetry.addData("09: ", "UltrasonicCM: " + robot.getDistanceCM());
//        telemetry.addData("10: ", "UltrasonicMM: " + robot.getDistanceMM());



    }

}
package org.firstinspires.ftc.team11248.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class BLUE_70pt extends OpMode {

    Robot11248 robot;
    VoltageSensor voltageSensor;
    @Override
    public void init() {
        robot = new Robot11248(hardwareMap, telemetry);
        robot.init(); //Sets servos to right position.
        //robot.activateColorSensors();
        robot.calibrateGyro(); //SETS ANGLE TOO 0 (BEFORE ANY MOVEMENT)


    }

    @Override
    public void loop() {

        robot.driveWithGyro(0,0,180);

    }
}

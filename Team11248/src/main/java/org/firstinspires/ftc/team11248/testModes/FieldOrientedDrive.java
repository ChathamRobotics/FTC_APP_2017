package org.firstinspires.ftc.team11248.testModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team11248.Robot11248;

/**
 * Created by Tony_Air on 3/1/17.
 */


@TeleOp(name = "Field Oriented Drive")
public class FieldOrientedDrive extends OpMode {

    Robot11248 robot;

    @Override
    public void init() {
        robot = new Robot11248(hardwareMap, telemetry);
        robot.init(); //Sets servos to right position.

    }

    @Override
    public void loop() {
        robot.driveWithFixedAngle(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                359 - robot.getGyroAngle());
    }


}

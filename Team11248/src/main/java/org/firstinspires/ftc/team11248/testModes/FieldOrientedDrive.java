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

    int targetAngle = 0;
    int increment = 5;
    boolean prevDPU, prevDPD;

    @Override
    public void init() {
        robot = new Robot11248(hardwareMap, telemetry);
        robot.init(); //Sets servos to right position.
        robot.calibrateGyro();

    }

    @Override
    public void loop() {

        robot.driveWithGyro(gamepad1.left_stick_x, -gamepad1.left_stick_y, (gamepad1.a?180:0));

        if(gamepad1.dpad_down && !prevDPD && targetAngle - increment > 0) targetAngle -= increment;
        prevDPD = gamepad1.dpad_down;

        if(gamepad1.dpad_up && !prevDPU && targetAngle + increment < 359) targetAngle += increment;
        prevDPU = gamepad1.dpad_up;

        telemetry.addData("targetAngle", targetAngle);
    }


}

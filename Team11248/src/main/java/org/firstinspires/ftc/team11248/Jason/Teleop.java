package org.firstinspires.ftc.team11248.Jason;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by tonytesoriero on 9/1/17.
 */


@TeleOp(name = "JASONSTELEOP", group = "General")

public class Teleop extends OpMode{

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.calibrateGyro();
    }

    @Override
    public void loop() {

        robot.setMotorPower(gamepad1.left_stick_x);

        if(gamepad1.a){
            robot.setServoPosition(.1104);
        }else{
            robot.setServoPosition(.91104);
        }

        telemetry.addData("TELEOP", "Heading: " + robot.getGyroAngle());
        telemetry.addData("TELEOP", "ODS: " + robot.getlight());
        telemetry.addData("TELEOP", "touch: " + robot.crocchow());

    }
}

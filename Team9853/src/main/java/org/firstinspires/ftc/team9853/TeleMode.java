package org.firstinspires.ftc.team9853;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.chathamrobotics.ftcutils.TeleOpMode;

import java.util.Map;

/**
 * teleop opmode
 */
@TeleOp(name = "Driving", group = "General")

public class TeleMode extends TeleOpMode {
    private DcMotor lift, sweeper, belt, shooter;
    private Servo liftToggle;

    @Override
    public void init() {
        super.init();

        lift = hardwareMap.dcMotor.get("Lift");
        sweeper = hardwareMap.dcMotor.get("CollectorSweeper");
        belt = hardwareMap.dcMotor.get("CollectorBelt");
        shooter = hardwareMap.dcMotor.get("Shooter");


        liftToggle = hardwareMap.servo.get("LiftToggle");
    }

    /*
         * Called continuously while opmode is active
         */
    @Override
    public void loop() {
        // Drive
        if(gamepad1.dpad_up){driver.offsetAngle = driver.FRONT_OFFSET;}
        if(gamepad1.dpad_left){driver.offsetAngle = driver.LEFT_OFFSET;}
        if(gamepad1.dpad_down){driver.offsetAngle = driver.BACK_OFFSET;}
        if(gamepad1.dpad_right){driver.offsetAngle = driver.RIGHT_OFFSET;}
        //Sets angle shift based on dpad
        if (gamepad1.dpad_up)
            driver.setOffsetAngle(0);
        else if (gamepad1.dpad_left)
            driver.setOffsetAngle(Math.PI/2);
        else if (gamepad1.dpad_down)
            driver.setOffsetAngle(Math.PI);
        else if (gamepad1.dpad_right)
            driver.setOffsetAngle(3*Math.PI/2);

        driver.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, true);

        lift.setPower(-gamepad2.left_stick_y);

        belt.setPower(gamepad2.right_stick_y);
        sweeper.setPower(gamepad2.right_stick_y);
        shooter.setPower(-gamepad2.right_trigger);

        if(gamepad2.y) {
            liftToggle.setPosition(0);
        }



        debug();
    }
}

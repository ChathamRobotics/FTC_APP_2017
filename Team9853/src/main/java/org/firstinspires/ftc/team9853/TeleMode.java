package org.firstinspires.ftc.team9853;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.chathamrobotics.ftcutils.OmniWheelDriver;
import org.chathamrobotics.ftcutils.TeleOpMode;

/**
 * teleop opmode
 */
@TeleOp(name = "Driving", group = "General")

public class TeleMode extends TeleOpMode {
    private DcMotor lift, sweeper, belt, shooter;
    private Servo leftLiftToggle;
    private Servo rightLiftToggle;

    static double toggleDownPosition = 0.2;
    static double toggleUpPosition = 1;

    @Override
    public void init() {
        super.init();

        lift = hardwareMap.dcMotor.get("Lift");
        sweeper = hardwareMap.dcMotor.get("Sweeper");
        belt = hardwareMap.dcMotor.get("Belt");
        shooter = hardwareMap.dcMotor.get("Shooter");


        leftLiftToggle = hardwareMap.servo.get("LeftLiftToggle");
        rightLiftToggle = hardwareMap.servo.get("RightLiftToggle");

        telemetry.addLine("Hardware initialized");
        telemetry.addLine("Press play to start");

        // account for log press
//        gameController2.y.setDelay(250);
//        gameController2.b.setDelay(250);
    }

    @Override
    public void start() {
        super.start();

        leftLiftToggle.setPosition(toggleDownPosition);
        rightLiftToggle.setPosition(toggleDownPosition);
    }

    /*
             * Called continuously while opmode is active
             */
    @Override
    public void loop() {
        // Drive
        if(gamepad1.dpad_up){driver.offsetAngle = OmniWheelDriver.FRONT_OFFSET;}
        if(gamepad1.dpad_left){driver.offsetAngle = OmniWheelDriver.LEFT_OFFSET;}
        if(gamepad1.dpad_down){driver.offsetAngle = OmniWheelDriver.BACK_OFFSET;}
        if(gamepad1.dpad_right){driver.offsetAngle = OmniWheelDriver.RIGHT_OFFSET;}

        driver.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x * .75, false);

        // lift
        lift.setPower(-gamepad2.left_stick_y);

        // Collecting
        belt.setPower(-gamepad2.right_stick_y);
        sweeper.setPower(-gamepad2.right_stick_y);

        // Shooting. the trigger value will always be positive
        shooter.setPower(-gamepad2.right_trigger);

        // Lift toggler
        if(gamepad2.y && gamepad2.b){
            if(leftLiftToggle.getPosition() == toggleUpPosition) {
                leftLiftToggle.setPosition(toggleDownPosition);
                rightLiftToggle.setPosition(toggleDownPosition);
            } else {
                leftLiftToggle.setPosition(toggleUpPosition);
                rightLiftToggle.setPosition(toggleUpPosition);
            }
        }

//        debug();
    }
}

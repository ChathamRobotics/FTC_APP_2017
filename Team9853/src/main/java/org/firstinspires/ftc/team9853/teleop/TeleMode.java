package org.firstinspires.ftc.team9853.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.chathamrobotics.ftcutils.OmniWheelDriver;
import org.chathamrobotics.ftcutils.Robot;
import org.chathamrobotics.ftcutils.TeleOpMode;
import org.firstinspires.ftc.team9853.Robot9853;

/**
 * teleop opmode
 */
@TeleOp(name = "Driving", group = "General")

public class TeleMode extends TeleOpMode {
//    CONSTANTS
    static double toggleDownPosition = 0.2;
    static double toggleUpPosition = 1;

//    COMPONENTS
    public Robot9853 robot;

//    METHODS
    @Override
    public Robot buildRobot() {
        return new Robot9853(hardwareMap, telemetry);
    }

    /**
     * Called continuously while opmode is active
     */
    @Override
    public void loop() {
        // Drive
        if(gamepad1.dpad_up){robot.changeFront(Robot9853.SideOfRobot.FRONT);}
        if(gamepad1.dpad_left){robot.changeFront(Robot9853.SideOfRobot.LEFT);}
        if(gamepad1.dpad_down){robot.changeFront(Robot9853.SideOfRobot.BACK);}
        if(gamepad1.dpad_right){robot.changeFront(Robot9853.SideOfRobot.RIGHT);}

        robot.teleopDrive(gamepad1);

        // lift
        robot.lift.setPower(-gamepad2.left_stick_y);

        // Collecting
        robot.belt.setPower(-gamepad2.right_stick_y);
        robot.sweeper.setPower(-gamepad2.right_stick_y);

        // Shooting. the trigger value will always be positive
        robot.shooter.setPower(-gamepad2.right_trigger);

        // Lift toggler
//        if(gamepad2.y && gamepad2.b){
//            if(leftLiftToggle.getPosition() == toggleUpPosition) {
//                leftLiftToggle.setPosition(toggleDownPosition);
//                rightLiftToggle.setPosition(toggleDownPosition);
//            } else {
//                leftLiftToggle.setPosition(toggleUpPosition);
//                rightLiftToggle.setPosition(toggleUpPosition);
//            }
//        }

        robot.debug();
    }
}

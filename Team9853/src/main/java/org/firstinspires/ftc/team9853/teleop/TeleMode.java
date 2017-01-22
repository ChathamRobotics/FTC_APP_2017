package org.firstinspires.ftc.team9853.teleop;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.chathamrobotics.ftcutils.OmniWheelDriver;
import org.chathamrobotics.ftcutils.Robot;
import org.chathamrobotics.ftcutils.TeleOpMode;
import org.firstinspires.ftc.team9853.Robot9853;
import org.firstinspires.ftc.team9853.opmodes.Tele;

/**
 * teleop opmode
 */
@TeleOp(name = "Driving", group = "General")

public class TeleMode extends Tele {
//    CONSTANTS     //
    static double toggleDownPosition = 0.2;
    static double toggleUpPosition = 1;

//    METHODS       //

    /**
     * Called continuously while opmode is active
     */
    @Override
    public void loop() {
        if(this.robot == null) robot = new Robot9853(hardwareMap, telemetry);

        // Drive
        if(gamepad1.dpad_up){robot.changeFront(Robot.Side.FRONT);}
        if(gamepad1.dpad_left){robot.changeFront(Robot.Side.LEFT);}
        if(gamepad1.dpad_down){robot.changeFront(Robot.Side.BACK);}
        if(gamepad1.dpad_right){robot.changeFront(Robot.Side.RIGHT);}

        // Use drives with controller values
        this.robot.teleopDrive(gamepad1);

        // lift
        this.robot.lift.setPower(-gamepad2.left_stick_y);

        // Collecting
        this.robot.setCollectorPower(-gamepad2.right_stick_y);

        // Shooting. the trigger value will always be positive
        this.robot.shooter.setPower(gamepad2.right_trigger);

        // Lift toggler
        if(gamepad2.y && gamepad2.b){
            this.robot.toggleLift();
        }

        robot.debug();
    }
}

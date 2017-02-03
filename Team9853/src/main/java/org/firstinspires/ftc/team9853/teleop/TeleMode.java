package org.firstinspires.ftc.team9853.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.chathamrobotics.ftcutils.Robot;
import org.firstinspires.ftc.team9853.opmodes.Tele9853;

/**
 * teleop opmode
 */
@TeleOp(name = "Driving", group = "General")

public class TeleMode extends Tele9853 {
//    CONSTANTS     //
    static double toggleDownPosition = 0.2;
    static double toggleUpPosition = 1;

//    METHODS       //

    /**
     * Called continuously while opmode is active
     */
    @Override
    public void loop() {
        // Drive
        if(gamepad1.dpad_up){robot().changeFront(Robot.Side.FRONT);}
        if(gamepad1.dpad_left){robot().changeFront(Robot.Side.LEFT);}
        if(gamepad1.dpad_down){robot().changeFront(Robot.Side.BACK);}
        if(gamepad1.dpad_right){robot().changeFront(Robot.Side.RIGHT);}

        // Use drives with controller values
        robot().teleopDrive(gamepad1);

        // lift
        robot().setLiftPower(-gamepad2.left_stick_y);

        // Collecting
        robot().setCollectorPower(-gamepad2.right_stick_y);

        // Shooting. the trigger value will always be positive
        this.robot().shoot(gamepad2.right_trigger);

        // Lift toggler
        if(gamepad2.y && gamepad2.b){
            this.robot().toggleLift();
        }

        robot().debug();
    }
}

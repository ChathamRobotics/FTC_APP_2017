package org.firstinspires.ftc.team9853.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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


    boolean prev1B = false;
    boolean prev1X = false;


//    METHODS       //

    /**
     * Called continuously while opmode is active
     */
    @Override
    public void loop() {

        // # GAMEPAD 1 #

        // Drive
        if(gamepad1.dpad_up){robot().changeFront(Robot.Side.FRONT);}
        if(gamepad1.dpad_left){robot().changeFront(Robot.Side.LEFT);}
        if(gamepad1.dpad_down){robot().changeFront(Robot.Side.BACK);}
        if(gamepad1.dpad_right){robot().changeFront(Robot.Side.RIGHT);}

        // Set driving mode
        if(gamepad1.x && !prev1B) robot().toggleDriftMode();
        prev1X = gamepad1.x;

        // Use drives with controller values
        robot().teleopDrive(gamepad1);

        // Lift toggler
        if(gamepad1.b && !prev1B) this.robot().toggleLift();
        prev1B = gamepad1.b;

        // lift
        if (gamepad1.right_trigger > 0)
            robot().setLiftPower(gamepad1.right_trigger);
        else if (gamepad1.left_trigger > 0)
            robot().setLiftPower(-gamepad1.left_trigger);
        else
            robot().setLiftPower(0);


        // # GAMEPAD 2 #

        // Collecting
        if(Math.abs(gamepad2.right_stick_y)>0) robot().setCollectorPower(-gamepad2.right_stick_y);
        else if(gamepad2.a) robot().setBeltPower(-.75);
        else if(gamepad2.y) robot().setBeltPower(.75);
        else robot().setBeltPower(0);


        // Shooting. the trigger value will always be positive
        if(gamepad2.right_trigger > 0) robot().setShooterPower();
        else if (robot().isShooterRunning()) robot().goToShooterPos0();




        robot().debug();
    }
}
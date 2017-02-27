package org.firstinspires.ftc.team11248.testModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by Tony_Air on 2/24/17.
 */

@TeleOp(group = "", name = "EncoderTest")
public class EncoderTest extends OpMode {

    DcMotor Shooter, Shooter1;
    final double TOLERANCE = .05;
    boolean isRunning = false;
    @Override
    public void init() {
        Shooter = hardwareMap.dcMotor.get("ShooterR");
        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Shooter1 = hardwareMap.dcMotor.get("ShooterL");
        Shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

//        if(gamepad2.right_trigger > TOLERANCE){
//            Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            Shooter.setPower(1);
//            isRunning = true;
//
//        } else if (isRunning) {
//            Shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            Shooter.setTargetPosition(Shooter.getCurrentPosition() + (1440-(Shooter.getCurrentPosition()%1440)));
//            isRunning = false;
//        }
//


        telemetry.addData("Position: ", Shooter.getCurrentPosition());
        telemetry.addData("Mod: ", Shooter1.getCurrentPosition());
        telemetry.addData("Trigger: ", gamepad2.right_trigger);

    }
}

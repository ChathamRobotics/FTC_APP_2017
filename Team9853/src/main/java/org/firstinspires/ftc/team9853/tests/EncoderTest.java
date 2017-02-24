package org.firstinspires.ftc.team9853.tests;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by Tony_Air on 2/24/17.
 */

@TeleOp(group = "", name = "EncoderTest")
public class EncoderTest extends OpMode {

    DcMotor Shooter;
    final double TOLERANCE = .05;
    boolean isRunning = false;
    @Override
    public void init() {
        Shooter = hardwareMap.dcMotor.get("Shooter");
        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        if(gamepad2.right_trigger > TOLERANCE){
            Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Shooter.setPower(1);
            isRunning = true;

        } else if (isRunning) {
            Shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Shooter.setTargetPosition(Shooter.getCurrentPosition() + (1440-(Shooter.getCurrentPosition()%1440)));
            isRunning = false;
        }



        telemetry.addData("Position: ", Shooter.getCurrentPosition());
        telemetry.addData("Mod: ", Shooter.getCurrentPosition() % 1440);
        telemetry.addData("Trigger: ", gamepad2.right_trigger);

    }
}

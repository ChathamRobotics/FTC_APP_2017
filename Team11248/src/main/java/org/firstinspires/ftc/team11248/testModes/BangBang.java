package org.firstinspires.ftc.team11248.testModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Tony_Air on 1/25/17.
 */

@TeleOp(name = "BangBangTest")
public class BangBang extends OpMode {

    private DcMotor motor;

    private double rpm, doubleSpeed, elapsedTime;
    private long lastTime;
    private long lastEncoder, currentEncoder;

    private final double tolerance = .015;
    private final int ticksPerRevolution = 1440;


    @Override
    public void init() {

        motor = hardwareMap.dcMotor.get("shooterL");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastEncoder = motor.getCurrentPosition();

    }

    @Override
    public void loop() {

        motor.setPower(1);
        bangBang(0, false);

        telemetry.addData("Power: ", doubleSpeed);
        telemetry.addData("RPM: ", rpm);
        telemetry.addData("Position: ", currentEncoder);
        telemetry.addData("Minutes: ", elapsedTime);
        telemetry.addData("Position: ", currentEncoder);


    }


    public void bangBang(double targetPower, boolean isBangBang){


        long now = System.nanoTime();
        double power = targetPower;

        elapsedTime = (now - lastTime)/60000000000L;
        currentEncoder = motor.getCurrentPosition();

        rpm = (currentEncoder-lastEncoder)/ticksPerRevolution / elapsedTime;
        doubleSpeed = rpm/152;


        if(isBangBang) {
            if (doubleSpeed >= (targetPower + tolerance)) {
                power = targetPower - .05;

            } else if (doubleSpeed <= (targetPower - tolerance)) {
                power = targetPower + .05;
            }

            motor.setPower(Range.clip(power, -1, 1));
        }


        lastEncoder = currentEncoder;
        lastTime = now;
    }
}

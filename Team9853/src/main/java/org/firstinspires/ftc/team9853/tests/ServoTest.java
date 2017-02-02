package org.firstinspires.ftc.team9853.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.team9853.opmodes.Tele9853;

import java.util.Map;

/**
 * Created by storm on 12/5/2016.
 */

@TeleOp(name = "Test: Servo", group = "Test")

@Disabled

public class ServoTest extends Tele9853 {
    public Servo[] servos;

    public ServoTest() {}

    @Override
    public void loop() {
        if(gamepad1.a) {
            for (long endtime = System.currentTimeMillis() + 250; System.currentTimeMillis() < endtime;) {

            }

            for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
                Servo servo = entry.getValue();

                servo.setPosition(Range.clip(servo.getPosition() + .1, 0, 1));
                telemetry.addData(entry.getKey(), servo.getPosition());
            }
        }

        if(gamepad1.b) {
            for (long endtime = System.currentTimeMillis() + 250; System.currentTimeMillis() < endtime;) {

            }

            for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
                Servo servo = entry.getValue();

                servo.setPosition(Range.clip(servo.getPosition() - .1, 0, 1));
                telemetry.addData(entry.getKey(), servo.getPosition());
            }
        }

        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue().getPosition());
        }
    }
}

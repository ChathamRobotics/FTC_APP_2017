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

//@Disabled

public class ServoTest extends Tele9853 {
    boolean lastA, lastB;

    public Servo[] servos;

    public ServoTest() {}

    @Override
    public void loop() {
        if(gamepad1.a && gamepad1.a != lastA) {
            for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
                Servo servo = entry.getValue();

                servo.setPosition(Range.clip(servo.getPosition() + .1, 0, 1));
            }
        }

        if(gamepad1.b && gamepad1.b != lastB) {

            for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
                Servo servo = entry.getValue();

                servo.setPosition(0);
//                servo.setPosition(Range.clip(servo.getPosition() - .1, 0, 1));
            }
        }

        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue().getPosition());
        }

        lastA = gamepad1.a;
        lastB = gamepad1.b;
    }
}

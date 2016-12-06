package org.firstinspires.ftc.team9853;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.chathamrobotics.ftcutils.TeleOpMode;

import java.util.Map;

/**
 * Created by storm on 12/5/2016.
 */

@TeleOp(name = "ServoTest", group = "Test")

public class ServoTest extends TeleOpMode {
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

        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue().getPosition());
        }
    }
}

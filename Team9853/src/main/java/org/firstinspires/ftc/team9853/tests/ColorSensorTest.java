package org.firstinspires.ftc.team9853.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.chathamrobotics.ftcutils.MRColorSensorV2;
import org.firstinspires.ftc.team9853.opmodes.Tele;

/**
 * test color sesnsors
 */

@TeleOp(name = "Test: ColorSensor", group = "Test")

//@Disabled

public class ColorSensorTest extends Tele {
    private MRColorSensorV2 sensor;

    @Override
    public void init() {
        super.init();
        sensor = new MRColorSensorV2(hardwareMap.i2cDevice.get("color") , 0x3c);
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            try { robot().doUntil(250); } catch (Exception e) {
                // Do nothing
            }

            sensor.enableLed(! sensor.isActive());
        }



            telemetry.addData(sensor.getDeviceName(), sensor.getColorNumber());
    }
}

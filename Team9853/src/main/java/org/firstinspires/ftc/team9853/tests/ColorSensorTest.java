package org.firstinspires.ftc.team9853.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDevice;

import org.chathamrobotics.ftcutils.MRColorSensorV2;
import org.chathamrobotics.ftcutils.TeleOpMode;

import java.util.HashMap;
import java.util.Map;

/**
 * Created by storm on 12/5/2016.
 */

@TeleOp(name = "Test: ColorSensor", group = "Test")

@Disabled

public class ColorSensorTest extends TeleOpMode {
    private HashMap<String, MRColorSensorV2> sensors;

    public ColorSensorTest() {
        int address = 0x3c;

        sensors = new HashMap<String, MRColorSensorV2>();

        for(Map.Entry<String, I2cDevice> entry : hardwareMap.i2cDevice.entrySet()) {
            sensors.put(entry.getKey(), new MRColorSensorV2(entry.getValue(), address));

            address++;
        }
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            for (long endtime = System.currentTimeMillis() + 250; System.currentTimeMillis() < endtime;) {

            }

            for(Map.Entry<String, MRColorSensorV2> entry : sensors.entrySet()) {
                entry.getValue().enableLed(! entry.getValue().isActive());
            }
        }


        for(Map.Entry<String, MRColorSensorV2> entry : sensors.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue().getColorNumber());
        }
    }
}

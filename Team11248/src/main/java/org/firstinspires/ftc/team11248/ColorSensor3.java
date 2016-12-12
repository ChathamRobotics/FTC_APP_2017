package org.firstinspires.ftc.team11248;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDevice;

import org.chathamrobotics.ftcutils.MRColorSensorV2;

/**
 * Created by Tony_Air on 12/6/16.
 */
@TeleOp(name = "SetColorAdress")

public class ColorSensor3 extends OpMode {

    public byte COLOR_SENSOR1_ADDR = 0x3C;
    public byte COLOR_SENSOR2_ADDR = 0x3D;
    public byte COLOR_SENSOR3_ADDR = 0x3E;

    MRColorSensorV2 color1,color2,color3;




    @Override
    public void init() {
        color1 = new MRColorSensorV2(hardwareMap.i2cDevice.get("color1"), COLOR_SENSOR1_ADDR);
        color2 = new MRColorSensorV2(hardwareMap.i2cDevice.get("color2"), COLOR_SENSOR2_ADDR);
        color3 = new MRColorSensorV2(hardwareMap.i2cDevice.get("color3"), COLOR_SENSOR3_ADDR);

        color1.enableLed(true);
        color2.enableLed(true);
        color3.enableLed(true);
    }

    @Override
    public void loop() {

        telemetry.addData("01: ", "R: " + color1.red() + " G: " + color1.green() + " B: " + color1.blue());
        telemetry.addData("02: ", "R: " + color2.red() + " G: " + color2.green() + " B: " + color2.blue());
        telemetry.addData("03: ", "R: " + color3.red() + " G: " + color3.green() + " B: " + color3.blue());

    }
}

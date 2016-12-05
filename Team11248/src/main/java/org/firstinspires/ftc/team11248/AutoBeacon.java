package org.firstinspires.ftc.team11248;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.chathamrobotics.ftcutils.MRColorSensorV2;

/**
 * Created by tonytesoriero on 10/29/16.
 */
//@Autonomous(name = "AutoB")
//
//public class AutoBeacon extends OpMode {
//
//    public Robot11248 robot;
//
//    public void runOpMode(){
//
//        //Initializes all sensors and motors
//        DcMotor[] motors = new DcMotor[8];
//        Servo[] servos = new Servo[1];
//        MRColorSensorV2[] colors = new MRColorSensorV2[3];
//        for(int i = 0; i < motors.length; i++)
//            motors[i] = hardwareMap.dcMotor.get(Robot11248.MOTOR_LIST[i]);
//        for(int i = 0; i < servos.length; i++)
//            servos[i] = hardwareMap.servo.get(Robot11248.SERVO_LIST[i]);
////      for(int i = 0; i < colors.length; i++)
////            colors[i] = new MRColorSensorV2(hardwareMap.i2cDevice.get(Robot11248.COLOR_LIST[i]));
//        colors[2] = new MRColorSensorV2(hardwareMap.i2cDevice.get(Robot11248.COLOR_LIST[2]));
//        robot = new Robot11248(motors,servos,colors,telemetry);
//    }
//
//    public void init() {
//        runOpMode();
//    }
//
//    public void loop() {
//        telemetry.addData("isRed", "stuff:=" + robot.getColor(3).getColorNumber());
//    }
//}
@TeleOp(name = "ColorRead")

public class AutoBeacon extends LinearOpMode {

    ColorSensor colorSensor;    // Hardware Device Object


    @Override
    public void runOpMode() {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.colorSensor.get("color3");

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            // check the status of the x button on either gamepad.
            bCurrState = gamepad1.x;

            // check for button state transitions.
            if ((bCurrState == true) && (bCurrState != bPrevState))  {

                // button is transitioning to a pressed state. So Toggle LED
                bLedOn = !bLedOn;
                colorSensor.enableLed(bLedOn);
            }

            // update previous state variable.
            bPrevState = bCurrState;

            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
        }
    }
}

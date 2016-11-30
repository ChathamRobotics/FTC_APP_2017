package org.firstinspires.ftc.team11248;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.chathamrobotics.ftcutils.MRColorSensorV2;

/**
 * Created by tonytesoriero on 10/29/16.
 */
@Autonomous(name = "AutoB")

public class AutoBeacon extends LinearOpMode {

    public Robot11248 robot;

    @Override
    public void runOpMode() throws InterruptedException {

        //Initializes all sensors and motors
        DcMotor[] motors = new DcMotor[8];
        Servo[] servos = new Servo[1];
        MRColorSensorV2[] colors = new MRColorSensorV2[3];
        for(int i = 0; i < motors.length; i++)
            motors[i] = hardwareMap.dcMotor.get(Robot11248.MOTOR_LIST[i]);
        for(int i = 0; i < servos.length; i++)
            servos[i] = hardwareMap.servo.get(Robot11248.SERVO_LIST[i]);
//      for(int i = 0; i < colors.length; i++)
//            colors[i] = new MRColorSensorV2(hardwareMap.i2cDevice.get(Robot11248.COLOR_LIST[i]));
        colors[2] = new MRColorSensorV2(hardwareMap.i2cDevice.get(Robot11248.COLOR_LIST[2]));
        robot = new Robot11248(motors,servos,colors,telemetry);
        robot.init();

        waitForStart();
        while (opModeIsActive()) {
            //sleep(10000); //wait idk why

            //robot.getTelemetry().addData("isRed", "stuff:=" + robot.getColor(3).isRed());

            break;

        }
    }
}

package org.firstinspires.ftc.team11248.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.chathamrobotics.ftcutils.MRColorSensorV2;
import org.firstinspires.ftc.team11248.Robot11248;

/**
 * Team 11248 Shooter Autonomous
 */
@Autonomous(name = "AutoShootRED")
public class AutoShootRED extends LinearOpMode{

    /**
     * The robot being controlled.
     */
    private Robot11248 robot;

    public static double LIFT_UP = 0;
    public static double LIFT_DOWN = 1;

    //Time spent driving forward in milliseconds
    //private long timeDriving = 3000;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initializes all sensors and motors
        DcMotor[] motors = new DcMotor[8];
        Servo[] servos = new Servo[2];
        I2cDevice color = hardwareMap.i2cDevice.get(Robot11248.COLOR);
        GyroSensor gyro = hardwareMap.gyroSensor.get(Robot11248.GYRO);
        OpticalDistanceSensor line = hardwareMap.opticalDistanceSensor.get(Robot11248.LINE);

        for (int i = 0; i < motors.length; i++)
            motors[i] = hardwareMap.dcMotor.get(Robot11248.MOTOR_LIST[i]);
        for (int i = 0; i < servos.length; i++)
            servos[i] = hardwareMap.servo.get(Robot11248.SERVO_LIST[i]);

        robot = new Robot11248(motors, servos, color, gyro, line,  telemetry);
        robot.init(); //Sets servos to right position.

        waitForStart();

        while (opModeIsActive()) {

            sleep(10000);

            robot.driveold(0,.8,0);
            sleep(1200);

            //drive(0,0);
            robot.stop();
            sleep(500);

            robot.shooterOn();
            sleep(750);

            robot.setConveyor(.3f);
            sleep(2500);

            robot.conveyorOff();
            robot.shooterOff();

            robot.driveold(0,1,0);
            sleep(1500);

            robot.stop();
            break;

        }
    }

}

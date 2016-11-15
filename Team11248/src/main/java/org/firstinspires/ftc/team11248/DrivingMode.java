package org.firstinspires.ftc.team11248;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Team 11248 TeleOp for real robot.
 */
@TeleOp(name = "Driving", group = "General")

public class DrivingMode extends DrivingModeOmni {

    public Robot11248 robot;

    @Override
    public void init() {
        //Initializes all sensors and motors
        DcMotor[] motors = new DcMotor[7];
        Servo[] servos = new Servo[2];
        for(int i = 0; i < motors.length; i++)
            motors[i] = hardwareMap.dcMotor.get(Robot11248.MOTOR_LIST[i]);
        for(int i = 0; i < servos.length; i++)
            servos[i] = hardwareMap.servo.get(Robot11248.SERVO_LIST[i]);
        robot = new Robot11248(motors,servos,telemetry);
    }

    @Override
    public void loop() {
        //Controls Wheels
        super.loop();

        //Sets liftarm up and down
        if (gamepad1.a)
            robot.switchLiftArm();

        //Sets shooter paddle up and down
        if (gamepad1.right_bumper)
            robot.switchPaddle();

        //Turns on/off shooter
        if (gamepad1.left_bumper)
            robot.shooterOn();
        else
            robot.shooterOff();

        //Sets arm motor to whatever right trigger is
        if (gamepad1.right_trigger > 0)
            robot.setLiftSpeed(gamepad1.right_trigger);
        else if (gamepad1.left_trigger > 0)
            robot.setLiftSpeed(-gamepad1.left_trigger);
        else
            robot.setLiftSpeed(0);
    }
}

package org.firstinspires.ftc.team11248.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team11248.Robot11248;

/**
 * Team 11248 TeleOp for real robot.
 */
@TeleOp(name = "DrivingFull", group = "General")
public class DriverFull extends DriverOmni {


    public Robot11248 robot;

     private boolean prevGP1y, prevGP2a, prevGP2x, prevGP2y;


    @Override
    public void init() {
        //Initializes all sensors and motors
        DcMotor[] motors = new DcMotor[8];
        Servo[] servos = new Servo[2];
        I2cDevice[] color = new I2cDevice[2];
        GyroSensor gyro = hardwareMap.gyroSensor.get("gyro");

        for(int i = 0; i < motors.length; i++)
            motors[i] = hardwareMap.dcMotor.get(Robot11248.MOTOR_LIST[i]);
        for(int i = 0; i < servos.length; i++)
            servos[i] = hardwareMap.servo.get(Robot11248.SERVO_LIST[i]);
        for(int i = 0; i < color.length; i++)
            color[i] = hardwareMap.i2cDevice.get(Robot11248.COLOR_LIST[i]);

        robot = new Robot11248(motors,servos, color, gyro, telemetry);
        robot.init(); //Sets servos to right position.

        robot.deactivateColorSensors();
    }

    @Override
    public void loop() {

       // ##GAMEPAD 1 CONTROLS ##

        //Controls Wheels
        if (gamepad1.dpad_up)
            robot.setOffsetAngle(0);
        else if (gamepad1.dpad_left)
            robot.setOffsetAngle(Robot11248.RIGHT_ANGLE);
        else if (gamepad1.dpad_down)
            robot.setOffsetAngle(2 * Robot11248.RIGHT_ANGLE);
        else if (gamepad1.dpad_right)
            robot.setOffsetAngle(3 * Robot11248.RIGHT_ANGLE);

        robot.driveold(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, true);

        //Sets liftarm up and down
        if (gamepad1.a)
            robot.moveLiftArmDown();
        else
            robot.moveLiftArmUp();


        if(gamepad1.b)
            robot.moveBeaconOut();
        else
            robot.moveBeaconIn();


        if (gamepad1.y && (gamepad1.y!=prevGP1y) )
            robot.switchSlow();

        prevGP1y = gamepad1.y;

        //Sets arm motor to whatever right trigger is
        if (gamepad1.right_trigger > 0)
            robot.setLiftSpeed(gamepad1.right_trigger);
        else if (gamepad1.left_trigger > 0)
            robot.setLiftSpeed(-gamepad1.left_trigger);
        else
            robot.setLiftSpeed(0);






        //## GAMEPAD 2 CONTROLS ##
        if (gamepad2.a && gamepad2.a != prevGP2a) {
            if(robot.getShooterOn())
                robot.shooterOff();
            else
                robot.shooterOn();
        }
        prevGP2a = gamepad2.a;


        if (gamepad2.b && gamepad2.y != prevGP2y) {
            if(robot.getShooterOn())
                robot.shooterOff();
            else
                robot.shooterReverse();
        }
        prevGP2y = gamepad2.y;


        if (gamepad2.x && gamepad2.x != prevGP2x) {

            if(robot.getShooterOn())
                robot.shooterOff();
            else
                robot.setShooter(-.5);
        }
        prevGP2x = gamepad2.x;


        if (gamepad2.right_trigger > 0)
            robot.setConveyor(-gamepad2.right_trigger);
        else if (gamepad2.left_trigger > 0)
            robot.setConveyor(gamepad2.left_trigger);
        else
            robot.setConveyor(0);

    }
}

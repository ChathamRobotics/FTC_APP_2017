package org.firstinspires.ftc.team11248.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.team11248.Robot11248;

/**
 * Team 11248 TeleOp for real robot.
 */
@TeleOp(name = "DrivingFull", group = "General")
public class DriverFull extends OpMode {


    public Robot11248 robot;

    private Gamepad prevGP1, prevGP2;
    boolean bangBang = false;


    @Override
    public void init() {

        //Initializes our Robot
        robot = new Robot11248(hardwareMap, telemetry);
        robot.init(); //Sets servos to right position.

        prevGP1 = new Gamepad();
        prevGP2 = new Gamepad();

        try {
            prevGP1.copy(gamepad1);
            prevGP2.copy(gamepad2);
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }

        robot.deactivateColorSensors();
        robot.setOffsetAngle(2 * Robot11248.RIGHT_ANGLE);
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
            robot.moveLiftArmUp();
        else
            robot.moveLiftArmDown();


        if(gamepad1.b)
            robot.moveBeaconOut();
        else
            robot.moveBeaconIn();

        robot.setFastMode(gamepad1.left_bumper);

        if (gamepad1.y && !prevGP1.y)
            robot.toggleSlow();
        telemetry.addData("",(robot.getIsSlow()?"SLOW":"FAST"));

        if (gamepad1.x && !prevGP1.x)
            robot.toggleDriftMode();
        telemetry.addData("",(robot.isDriftModeOn()?"DRIFT":"BREAK"));


        //Sets arm motor to whatever right trigger is
        if (gamepad1.right_trigger > 0)
            robot.setLiftSpeed(gamepad1.right_trigger);
        else if (gamepad1.left_trigger > 0)
            robot.setLiftSpeed(-gamepad1.left_trigger);
        else
            robot.setLiftSpeed(0);


        //Recaptures all previous values of Gampad 1 for debouncing
        try {
            prevGP1.copy(gamepad1);
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }





        //## GAMEPAD 2 CONTROLS ##
        if (gamepad2.a && !prevGP2.a) {
            if(robot.getShooterOn()) {
                robot.shooterOff();
               // bangBang = false;
            }else
              robot.setShooter(Robot11248.SHOOTER_SPEED); // bangBang = true;
        }



        if (gamepad2.b && !prevGP2.b) {
            if (robot.getShooterOn())
                robot.shooterOff();
            else
                robot.shooterReverse();
        }


        if (gamepad2.x && !prevGP2.x)
            robot.switchCollectorServo();

        telemetry.addData("",(robot.collectorClosed?"CLOSED":"OPEN"));
        telemetry.update();


        if (gamepad2.right_trigger > 0)
            robot.setConveyor(gamepad2.right_trigger * (robot.collectorClosed?1f:1f));
        else if (gamepad2.left_trigger > 0)
            robot.setConveyor(-gamepad2.left_trigger * (robot.collectorClosed?1f:1f));
        else
            robot.setConveyor(0);


        //Recaptures all previous values of Gamepad 2 for debouncing
        try {
            prevGP2.copy(gamepad2);
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }


    }
}

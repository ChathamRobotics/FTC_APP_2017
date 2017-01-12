package org.firstinspires.ftc.team9853;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.chathamrobotics.ftcutils.OmniWheelDriver;
import org.chathamrobotics.ftcutils.Robot;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

/**
 * class containing all of the robot functionality for the 2017 build
 */

public class Robot9853 extends Robot {
//    COMPONENTS    //

    protected OmniWheelDriver driver;

//    HARDWARE      //

    public DcMotor lift;
    public DcMotor sweeper;
    public DcMotor belt;
    public DcMotor shooter;

//    CONSTRUCTORS  //

    public Robot9853(HardwareMap hw, Telemetry tele) {
        super(hw, tele);
    }

//    METHODS       //

    /**
     * puts the robot in start position.
     */
    @Override
    public void start() {

    }

    /**
     * initializes hardware.
     */
    @Override
    public void initHardware() {
        driver = OmniWheelDriver.build(this.hardwareMap, this.telemetry);
        driver.silent = false;

        lift = hardwareMap.dcMotor.get("Lift");
        sweeper = hardwareMap.dcMotor.get("Sweeper");
        belt = hardwareMap.dcMotor.get("Belt");
        shooter = hardwareMap.dcMotor.get("Shooter");

        telemetry.addLine("Hardware initialized");
        telemetry.addLine("Press play to start");
    }

    /**
     * Changes the referenced front of the robot.
     * @param newFront  the front that should be referenced. (ex: if the left side of the robot should be the front call (changeFront(
     */
    public void changeFront(Side newFront) {
        this.driver.offsetAngle = newFront.angle;
    }

    /**
     * Drives the robot give then game pad readings
     * @param gamepad   the controller to get the readings from
     */
    public void teleopDrive(Gamepad gamepad) {
        this.driver.drive(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x * .75);
    }

    /**
     * Drives a specified angle
     * @param angle         the angle to drive in. Relative to the front in radians
     * @param speedModifier the speed modifier to use
     */
    public void driveAtAngle(double angle, double speedModifier) {
        this.driver.move(angle, 0, speedModifier);
    }
    public void driveAtAngle(double angle) {
        driveAtAngle(angle, 1);
    }

    /**
     * Drives in the direction specified by the x and y given
     * @param x             the x part of the point
     * @param y             the y part of the point
     * @param speedModifier the speed modifier to use
     */
    public void driveAtPoint(double x, double y, double speedModifier) {
        this.driver.drive(x, y, 0, speedModifier, false);
    }
    public void driveAtPoint(double x, double y) {
        driveAtPoint(x, y, 1);
    }

    /**
     * Drives forward.
     * @param speedModifier the speed modifier to use
     */
    public void driveForward(double speedModifier) {
        this.driveAtAngle(Math.PI / 2);
    }
    public void driveForward() {
        driveForward(1);
    }

    /**
     * Stops the robot from driving.
     */
    public void stopDriving() {
        this.driver.move(0,0,0);
    }
}

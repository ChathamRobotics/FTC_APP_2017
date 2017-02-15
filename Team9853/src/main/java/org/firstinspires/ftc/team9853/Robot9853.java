package org.firstinspires.ftc.team9853;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.chathamrobotics.ftcutils.OmniWheelDriver;
import org.chathamrobotics.ftcutils.Robot;
import org.chathamrobotics.ftcutils.hardware.MRColorSensorV3;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

/**
 * class containing all of the robot functionality for the 2017 build
 */

public class Robot9853 extends Robot {
//    CONSTANTS     //
    public static final double TOGGLE_UP_POSITION = 0.2;
    public static final double TOGGLE_DOWN_POSITION = 1;

    public static final double OPTICAL_LOW = .6;
    public static final double OPTICAL_HIGH = 1;

    public static final int GYRO_MIN_ANGLE = 2;

    public static final long RELOAD_TIME = 2500;
    public static final long SHOOT_TIME = 500;

    public static  final double SENSING_SPEED = .26;

//    COMPONENTS    //

    private OmniWheelDriver driver;

//    HARDWARE      //

    private DcMotor lift;
    private DcMotor sweeper;
    private DcMotor belt;
    private DcMotor shooter;
    private Servo liftToggle;
    private OpticalDistanceSensor leftLineSensor;
    private OpticalDistanceSensor centerLineSensor;
    private MRColorSensorV3 beaconSensor;
    private GyroSensor gyro;
    private TouchSensor touchSensor;

//    STATEFUL      //
    private long lastLiftToggle;

    public int startingHeading;

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
//        this.calibrateGyro();
//        this.startingHeading = this.gyro.getHeading();

        this.liftToggle.setPosition(TOGGLE_DOWN_POSITION);

        this.beaconSensor.setPassiveMode();

        this.log("Starting robot...");
    }

    /**
     * initializes hardware.
     */
    @Override
    public void initHardware() {
        this.driver = OmniWheelDriver.build(this.hardwareMap, this.telemetry);
        this.driver.silent = false;

        this.lift = hardwareMap.dcMotor.get("Lift");
        this.sweeper = hardwareMap.dcMotor.get("Sweeper");
        this.belt = hardwareMap.dcMotor.get("Belt");

        this.shooter = hardwareMap.dcMotor.get("Shooter");
        this.shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.liftToggle = hardwareMap.servo.get("LiftToggle");


        leftLineSensor = hardwareMap.opticalDistanceSensor.get("LeftLineSensor");
        centerLineSensor = hardwareMap.opticalDistanceSensor.get("CenterLineSensor");
        gyro = hardwareMap.gyroSensor.get("Gyro");
        beaconSensor = new MRColorSensorV3(hardwareMap.i2cDevice.get("BeaconSensor"), (byte) 0x3c);
        touchSensor = hardwareMap.touchSensor.get("Touch");

        this.telemetry.addLine("Hardware initialized");
        this.telemetry.addLine("Press play to start");
    }



    /**
     * Changes the referenced front of the robot.
     * @param newFront  the front that should be referenced. (ex: if the left side of the robot should be the front call (changeFront(
     */
    public void changeFront(Side newFront) {
        this.driver.offsetAngle = newFront.offset;
        this.log("NewFront", newFront.name);
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
        this.driveAtAngle(Math.PI / 2, speedModifier);
    }
    public void driveForward() {
        driveForward(1);
    }

    /**
     * Drives in the direction specified by the angle and maintains heading
     * @param angle         the direction to drive in
     * @param speedModifier the modifier for the speed
     * @param targetHeading the heading to follow
     * @return              whether the heading has been achieved.
     */
    public boolean driveWithHeading(double angle, double speedModifier, int targetHeading) {
        boolean atHeading = false;
        int currentHeading = gyro.getHeading();
        int net = currentHeading - targetHeading; //finds distance to target angle
        double rotation;

        if (Math.abs(net) > 180) { // if shortest path passes 0
            if (currentHeading > 180) //if going counterclockwise
                net = (currentHeading - 360) - targetHeading;

            else //if going clockwise
                net = (360 - targetHeading) + currentHeading;
        }

        // slows down as approaches angle with min threshold of .05
        // each degree adds/subtracts .95/180 values of speed
        rotation = Math.abs(net) * .85 / 180 + .15;

        if (net < 0) rotation *= -1; //if going clockwise, set rotation clockwise (-)

        if (Math.abs(net) > GYRO_MIN_ANGLE)
            driver.move(angle, speedModifier, rotation); //Drive with gyros rotation

        else {
            atHeading = true;
            driver.move(angle,speedModifier, rotation);
        }

        this.log("Current Heading", currentHeading);
        this.log("Target Heading", targetHeading);
        this.log("Target is to the", net > 0 ? "Left" : "Right");

        return atHeading;

//        int currentHeading = gyro.getHeading(),
//                headingDif = currentHeading - targetHeading;
//        boolean atHeading = false;
//        double requiredRotation;
//
//        if(Math.abs(headingDif) > 180) {
//            if(headingDif < 0) // clockwise
//                headingDif = (360 - targetHeading) + currentHeading;
//            else // counter clockwise
//                headingDif = (currentHeading - 360) - targetHeading;
//        }
//
//        // the heading dif should never be greater in magnitude than 180. neg is left and pos is right
//        // make the rotation proportional to 180
//        requiredRotation = headingDif / 180;
//
//        this.log("Current Heading", currentHeading);
//        this.log("Target Heading", targetHeading);
//        this.log("Target is to the", headingDif > 0 ? "Left" : "Right");
//
//        if(Math.abs(headingDif) > GYRO_MIN_ANGLE)
//            driver.move(angle, requiredRotation, speedModifier);
//        else {
//            driver.move(angle, 0, speedModifier);
//            atHeading = true;
//        }
//
//        return atHeading;
    }
    public boolean driveWithHeading(double angle, int targetHeading) {
        return driveWithHeading(angle, 1, targetHeading);
    }

    /**
     * Drives a angle for a period of time
     * @param angle         the direction to drive
     * @param speedModifier the speed modifier to use
     * @param duration      the amount of time to do for
     * @return              whether time is up
     */
    public boolean driveAtAngleFor(double angle, double speedModifier, long duration) {
        boolean result = doUntil(duration);

        driveAtAngle(angle, speedModifier);

        if(isTimerUp()) {
            stopDriving();
        }

        return result;
    }
    public boolean driveAtAngleFor(double angle, long duration) {
        return driveAtAngleFor(angle, 1, duration);
    }

    /**
     * Drives in the direction specified by the x and y given for the given time
     * @param x             the x part of the direction
     * @param y             the y part of the direction
     * @param speedModifier the speed modifier to use
     * @param duration      the amount of time to do for
     * @return              whether the time is up
     */
    public boolean driveAtPointFor(double x, double y, double speedModifier, long duration) {
        boolean result = doUntil(duration);

        driveAtPoint(x, y, speedModifier);

        if(isTimerUp()) {
            stopDriving();
        }

        return result;
    }
    public boolean driveAtPointFor(double x, double y, long duration) {return driveAtPointFor(x, y, 1, duration);}

    /**
     * Drives forward for the specified time
     * @param speedModifier the speed modifier to use
     * @param duration      the amount of time to do for
     * @return              whether time is up
     */
    public boolean driveForwardFor(double speedModifier, long duration) {
        boolean result = doUntil(duration);

        driveForward(speedModifier);

        if(isTimerUp()) {
            stopDriving();
        }

        return result;
    }
    public boolean driveForwardFor(long duration) {return driveForwardFor(1, duration);}

    /**
     * Drive in the direction specified by the angle while attempting to reach the target heading for the given duration
     * @param angle         the direction to travel in
     * @param speedModifier the speed modifier to use
     * @param targetHeading the heading to maintain
     * @param duration      the amount of time to do it for in ms
     * @return              whether time is up
     */
    public boolean driveWithHeadingFor(double angle, double speedModifier, int targetHeading, long duration) {
        boolean result = doUntil(duration);

        driveWithHeading(angle, speedModifier, targetHeading);

        if(isTimerUp()) {
            stopDriving();
        }

        return result;
    }
    public boolean driveWithHeadingFor(double angle, int targetHeading, long duration) {
        return driveWithHeadingFor(angle, 1, targetHeading, duration);
    }


    /**
     * drives at angle while condition is true
     * @param angle         the direction to drive in
     * @param speedModifier the speed modifier to use
     * @param con           the condition
     * @return              whether the condition was true
     */
    public boolean driveAtAngleWhile(double angle, double speedModifier, boolean con) {
        driveAtAngle(angle, speedModifier);

        if(! con) stopDriving();

        return con;
    }
    public boolean driveAtAngleWhile(double angle, boolean con) {
        return driveAtAngleWhile(angle, 1, con);
    }

    /**
     * drive in the direction specified by point while the condition is true
     * @param x             the x part of the direction
     * @param y             the y part of the direction
     * @param speedModifier the speed modifier to use
     * @param con           the condition
     * @return              whether the condition was true
     */
    public boolean driveAtPointWhile(double x, double y, double speedModifier, boolean con) {
        driveAtPoint(x, y, speedModifier);

        if(! con) stopDriving();

        return con;
    }
    public boolean driveAtPointWhile(double x, double y, boolean con) {
        return driveAtPointWhile(x, y, 1, con);
    }

    /**
     * drives forward while the condition is true
     * @param speedModifier the speed modifier to use
     * @param con           the condition
     * @return              whether the condition was true
     */
    public boolean driveForwardWhile(double speedModifier, boolean con) {
        driveForward(speedModifier);

        if(! con) stopDriving();

        return con;
    }
    public boolean driveForWhile(boolean con) {
        return driveForwardWhile(1, con);
    }

    /**
     * Drive in the direction specified by the angle while attempting to reach the target heading while  the condition is true
     * @param angle         the direction to travl in
     * @param speedModifier the speed modifier to use
     * @param targetHeading the heading to maintain
     * @param con           the condition
     * @return              whether the condition is true
     */
    public boolean driveWithHeadingWhile(double angle, double speedModifier, int targetHeading, boolean con) {
        driveWithHeading(angle, speedModifier, targetHeading);

        if(! con) stopDriving();
        return con;
    }
    public boolean driveWithHeadingWhile(double angle, int targetHeading, boolean con) {
        return driveWithHeadingWhile(angle, 1, targetHeading, con);
    }


    /**
     * Stops the robot from driving.
     */
    public void stopDriving() {
        this.driver.move(0,0,0);
    }

    /**
     * Sets the collection systems power.
     * @param power the power value to set the motors to
     */
    public void setCollectorPower(double power) {
        this.sweeper.setPower(power);
        this.belt.setPower(-power);
    }

    public void setBeltPower(double power) {
        this.belt.setPower(power);
    }

    /**
     * sets the lifts power
     * @param power the power value to set the motor to
     */
    public void setLiftPower(double power) {
        this.lift.setPower(power);
    }

    /**
     * toggle the lift servos position
     */
    public void toggleLift() {
        if(System.currentTimeMillis() >= lastLiftToggle + 250) {
            lastLiftToggle = System.currentTimeMillis();
            if(liftToggle.getPosition() == TOGGLE_UP_POSITION) {
                liftToggle.setPosition(TOGGLE_DOWN_POSITION);
            } else {
                liftToggle.setPosition(TOGGLE_UP_POSITION);
            }
        }
    }

    /**
     * Calibrate the gyro
     */
    public void calibrateGyro() {
        gyro.calibrate();


        while(gyro.isCalibrating()) {
            // Stall
        }
    }

    /**
     * whether the left line sensor sees the white line
     * @return whether is at line
     */
    public boolean isLeftAtLine() {
        double lightVal = this.leftLineSensor.getLightDetected();

        return lightVal >= OPTICAL_LOW && lightVal <= OPTICAL_HIGH;
    }

    /**
     * whether the center line sensor sees the white line
     * @return whether is at line
     */
    public boolean isCenterAtLine() {
        double lightVal = this.centerLineSensor.getLightDetected();

        return lightVal >= OPTICAL_LOW && lightVal <= OPTICAL_HIGH;
    }

    /**
     * whether the beacon is in range
     * @return whether beacon is readable
     */
    public boolean isBeaconInRange() {
        return beaconSensor.getColorNumber() != 0;
    }

    /**
     * determine whether or not the beacon is red
     * @return whether or not the beacon sensor is reading red
     */
    public boolean isBeaconRed() {
        return beaconSensor.getColorNumber() == 10;
    }

    /**
     * determine whether or not the beacon is blue
     * @return whether or not the beacon sensor is reading red
     */
    public boolean isBeaconBlue() {
        return beaconSensor.getColorNumber() == 3;
    }

    /**
     * determine whether or not the beacon is being pressed
     * @return whether the touch sensor is being pressed
     */
    public boolean isBeaconTouching() {
        // test all touch sensors
        for (Map.Entry<String, TouchSensor> entry: this.hardwareMap.touchSensor.entrySet()) {
            if(entry.getValue().isPressed()) return true;
        }
        return false;
    }

    /**
     * runs the shooter
     * @param power the power to set the shooter to
     */
    public void shoot(double power){
        this.shooter.setPower(power);
    }
    public void shoot() {
        shoot(.7);
    }

    /**
     * shoots until the time is up
     * @param power     the power to set the shooter to
     * @param endTime   the time to end
     * @return          whether time is up
     */
    public boolean shootFor(double power, long endTime) {
        shoot(power);

        if(System.currentTimeMillis() >= endTime) {
            shoot(0);
            return false;
        }

        return true;
    }
    public boolean shootFor(long endTime) {
        return shootFor(.7, endTime);
    }

    /**
     * runs the collector until time is up
     * @param endTime   the time to end
     * @return          whether time is up
     */
    public boolean reloadFor(long endTime) {
        setCollectorPower(-1);

        if(System.currentTimeMillis() >= endTime) {
            setCollectorPower(0);
            return false;
        }

        return true;
    }
}

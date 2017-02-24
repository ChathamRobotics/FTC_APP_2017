package org.firstinspires.ftc.team9853;

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
    public static final double TOGGLE_UP_POSITION = 0;
    public static final double TOGGLE_DOWN_POSITION = 1;

    public static final double OPTICAL_LOW = .6;
    public static final double OPTICAL_HIGH = 1;

    public static final int GYRO_MIN_ANGLE = 15;

    public static final long RELOAD_TIME = 2500;
    public static final long SHOOT_TIME = 500;

    public static  final double SENSING_SPEED = .32;

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
    public MRColorSensorV3 centerBeaconSensor;
    public MRColorSensorV3 leftBeaconSensor;
    public MRColorSensorV3 rightBeaconSensor;
    private GyroSensor gyro;

//    STATEFUL      //
    private double lastLiftToggle;
    private boolean isShooterRunning;
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
        // store the initial heading for later use
        waitForGyroCalibration();
        this.startingHeading = this.gyro.getHeading();

        // put the lift toggler down
        this.liftToggle.setPosition(TOGGLE_DOWN_POSITION);

        // set the beacon sensing system to passive
        this.centerBeaconSensor.setPassiveMode();
        this.leftBeaconSensor.setPassiveMode();
        this.rightBeaconSensor.setPassiveMode();

        this.logger.info("Starting robot...");
    }

    /**
     * initializes hardware.
     */
    @Override
    public void initHardware() {
        // Setup drivier
        driver = OmniWheelDriver.build(this.hardwareMap, this.telemetry);
        driver.silent = false;

        // collection system
        sweeper = hardwareMap.dcMotor.get("Sweeper");
        belt = hardwareMap.dcMotor.get("Belt");

        // lift system
        lift = hardwareMap.dcMotor.get("Lift");
        liftToggle = hardwareMap.servo.get("LiftToggle");

        // shooter system
        shooter = hardwareMap.dcMotor.get("Shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        isShooterRunning = false;
        //shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // line sensor system
        leftLineSensor = hardwareMap.opticalDistanceSensor.get("LeftLineSensor");
        centerLineSensor = hardwareMap.opticalDistanceSensor.get("CenterLineSensor");

        // gyro system
        gyro = hardwareMap.gyroSensor.get("Gyro");
        gyro.calibrate();

        // beacon sensor system
        centerBeaconSensor = new MRColorSensorV3(hardwareMap.i2cDevice.get("CenterBeaconSensor"), (byte) 0x3c);
        leftBeaconSensor = new MRColorSensorV3(hardwareMap.i2cDevice.get("LeftBeaconSensor"), (byte) 0x34);
        rightBeaconSensor = new MRColorSensorV3(hardwareMap.i2cDevice.get("RightBeaconSensor"), (byte) 0x60);

        this.telemetry.addLine("Hardware initialized");
        this.telemetry.addLine("Press play to start");
    }



    /**
     * Changes the referenced front of the robot.
     *
     * @param newFront  the front that should be referenced.
     */
    public void changeFront(Side newFront) {
        this.driver.offsetAngle = newFront.offset;
        this.logger.info("NewFront", newFront.name);
    }

    /**
     * toggles the drive motors mode from drifting to braking
     */

    public void toggleDriftMode(){
        this.driver.toggleDriftMode();
    }

    /**
     * Drives the robot give then game pad readings
     *
     * @param gamepad   the controller to get the readings from
     */
    public void teleopDrive(Gamepad gamepad) {
        this.driver.drive(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x * .75);
    }

    /**
     * Drives in the direction specified by the angle
     *
     * @param angle         the direction to drive. Relative to the front (in rads)
     * @param speedModifier the speed modified by multiplying it by this number
     */
    public void driveAtAngle(double angle, double speedModifier) {
        this.driver.move(angle, 0, speedModifier);
    }

    /**
     * Drives in the direction specified by the angle
     *
     * @param angle the direction to drive. Relative to the front (in rads)
     */
    public void driveAtAngle(double angle) {
        driveAtAngle(angle, 1);
    }

    /**
     * Drives in the direction specified by the x and y given
     *
     * @param x             the x part of the point
     * @param y             the y part of the point
     * @param speedModifier the speed modified by multiplying it by this number
     */
    public void driveAtPoint(double x, double y, double speedModifier) {
        this.driver.drive(x, y, 0, speedModifier, false);
    }

    /**
     * Drives in the direction specified by the x and y
     *
     * @param x the x part of the point
     * @param y the y part of the point
     */
    public void driveAtPoint(double x, double y) {
        driveAtPoint(x, y, 1);
    }

    /**
     * Drives forward.
     *
     * @param speedModifier the speed modified by multiplying it by this number
     */
    public void driveForward(double speedModifier) {
        this.driveAtAngle(Math.PI / 2, speedModifier);
    }

    /**
     * Drives forward.
     */
    public void driveForward() {
        driveForward(1);
    }

    /**
     * Drives in the direction specified by the angle and maintains heading
     *
     * @param angle         the direction to drive in
     * @param speedModifier the speed modified by multiplying it by this number
     * @param targetHeading the heading to try to maintain
     * @return              whether the heading has been achieved.
     */
    public boolean driveWithHeading(double angle, double speedModifier, int targetHeading) {
        targetHeading %= 360;

        boolean atHeading = false;
        int currentHeading = gyro.getHeading();
        //finds distance to target angle
        int headingDif = currentHeading - targetHeading;
        double requiredRotation;

        // If the shortest path to target passes through 0
        if (Math.abs(headingDif) > 180) {
            // If counterclockwise is shorter
            if (currentHeading > 180) headingDif = (currentHeading - 360) - targetHeading;
            // If clockwise is shorter
            else headingDif = (360 - targetHeading) + currentHeading;
        }

        // slows down as approaches angle with min threshold of .05
        // each degree adds/subtracts .95/180 values of speed
        requiredRotation = Math.abs(headingDif) * .85 / 180 + .15;

        //if going clockwise, set requiredRotation clockwise (-)
        if (headingDif < 0) requiredRotation *= -1;

        if (Math.abs(headingDif) > (speedModifier == 0 ? 2 : GYRO_MIN_ANGLE * speedModifier))
            //Drive with gyros requiredRotation
            driver.move(angle, requiredRotation, speedModifier);
        else {
            atHeading = true;
            driver.move(angle, 0, speedModifier);
        }

        // debug
        this.logger.debug("Current Heading", currentHeading, true);
        this.logger.debug("Target Heading", targetHeading, true);
        this.logger.debug("Distance to target heading", headingDif, true);
        this.logger.debug("Angular Speed Modifier", requiredRotation, true);
        this.logger.debug("Turning to the" + (headingDif > 0 ? "Left" : "Right"), true);


        return atHeading;
    }

    /**
     * Drives in the direction specified by the angle and maintains heading.
     * Assumes speed modifier is 1
     * @see Robot9853#driveWithHeading(double, double, int)
     *
     * @param angle         the direction to drive
     * @param targetHeading the heading to try to maintain
     * @return              whether the heading has been achieved.
     */
    public boolean driveWithHeading(double angle, int targetHeading) {
        return driveWithHeading(angle, 1, targetHeading);
    }

    /**
     * Drives a angle for the given period of time
     * @see Robot9853#driveAtAngle(double, double)
     *
     * @param angle         the direction to drive
     * @param speedModifier the speed modified by multiplying it by this number
     * @param duration      the amount of time to drive for
     * @return              whether duration has expired
     */
    public boolean driveAtAngleFor(double angle, double speedModifier, long duration) {
        boolean result = doUntil(duration);

        driveAtAngle(angle, speedModifier);

        if(isTimerUp()) {
            stopDriving();
        }

        return result;
    }

    /**
     * Drives a angle for the given period of time. Assumes speed modifier is 1
     * @see Robot9853#driveAtAngleFor(double, double, long)
     *
     * @param angle     the direction to drive
     * @param duration  the amount of time to drive for
     * @return          whether duration has expired
     */
    public boolean driveAtAngleFor(double angle, long duration) {
        return driveAtAngleFor(angle, 1, duration);
    }

    /**
     * calls driveAtPoint for the given period of time
     * @see Robot9853#driveAtPoint(double, double, double)
     *
     * @param x             the x part of the direction
     * @param y             the y part of the direction
     * @param speedModifier the speed modified by multiplying it by this number
     * @param duration      the amount of time to drive for
     * @return              whether duration has expired
     */
    public boolean driveAtPointFor(double x, double y, double speedModifier, long duration) {
        boolean result = doUntil(duration);

        driveAtPoint(x, y, speedModifier);

        if(isTimerUp()) {
            stopDriving();
        }

        return result;
    }

    /**
     * calls driveAtPoint for the given period of time. Assumes speed modifier is 1
     * @see Robot9853#driveAtPointFor(double, double, double, long)
     *
     * @param x         the x part of the direction
     * @param y         the y part of the direction
     * @param duration  the amount of time to drive for
     * @return          whether duration has expired
     */
    public boolean driveAtPointFor(double x, double y, long duration) {return driveAtPointFor(x, y, 1, duration);}

    /**
     * Drives forward for the specified time.
     * @see Robot9853#driveForward(double)
     *
     * @param speedModifier the speed modified by multiplying it by this number
     * @param duration      the amount of time to drive for
     * @return              whether duration has expired
     */
    public boolean driveForwardFor(double speedModifier, long duration) {
        boolean result = doUntil(duration);

        driveForward(speedModifier);

        if(isTimerUp()) {
            stopDriving();
        }

        return result;
    }

    /**
     * Drives forward for the specified time. Assumes speed modifier is 1
     * @see Robot9853#driveForwardFor(double, long)
     * @see Robot9853#driveForward(double)
     *
     * @param duration  the amount of time to drive for
     * @return          whether duration has expired
     */
    public boolean driveForwardFor(long duration) {return driveForwardFor(1, duration);}

    /**
     * Drive in the direction specified by the angle while attempting to reach the target heading
     * for the given duration
     * @see Robot9853#driveWithHeading(double, double, int)
     *
     * @param angle         the direction to travel in
     * @param speedModifier the speed modified by multiplying it by this number
     * @param targetHeading the heading to maintain
     * @param duration      the amount of time to drive for
     * @return              whether duration has expired
     */
    public boolean driveWithHeadingFor(double angle, double speedModifier, int targetHeading, long duration) {
        boolean result = doUntil(duration);

        driveWithHeading(angle, speedModifier, targetHeading);

        if(isTimerUp()) {
            stopDriving();
        }

        return result;
    }

    /**
     * Drive in the direction specified by the angle while attempting to reach the target heading
     * for the given duration. Assumes speed modifier is 1
     * @see Robot9853#driveWithHeading(double, double, int)
     * @see Robot9853#driveWithHeadingFor(double, double, int, long)
     *
     * @param angle         the direction to travel in
     * @param targetHeading the heading to maintain
     * @param duration      whether duration has expired
     * @return              whether duration has expired
     */
    public boolean driveWithHeadingFor(double angle, int targetHeading, long duration) {
        return driveWithHeadingFor(angle, 1, targetHeading, duration);
    }


    /**
     * drives at angle while condition is true.
     * @see Robot9853#driveAtAngle(double, double)
     *
     * @param angle         the direction to drive in
     * @param speedModifier the speed modified by multiplying it by this number
     * @param con           the condition
     * @return              whether the condition was true
     */
    public boolean driveAtAngleWhile(double angle, double speedModifier, boolean con) {
        driveAtAngle(angle, speedModifier);

        if(! con) stopDriving();

        return con;
    }

    /**
     * drives at angle while condition is true. Assumes speed modifier is 1
     * @see Robot9853#driveAtAngle(double, double)
     * @see Robot9853#driveAtAngleWhile(double, double, boolean)
     *
     * @param angle the direction to drive in
     * @param con   the condition
     * @return      whether the condition was true
     */
    public boolean driveAtAngleWhile(double angle, boolean con) {
        return driveAtAngleWhile(angle, 1, con);
    }

    /**
     * drive in the direction specified by point while the condition is true
     * @see Robot9853#driveAtPoint(double, double, double)
     *
     * @param x             the x part of the direction
     * @param y             the y part of the direction
     * @param speedModifier the speed modified by multiplying it by this number
     * @param con           the condition
     * @return              whether the condition was true
     */
    public boolean driveAtPointWhile(double x, double y, double speedModifier, boolean con) {
        driveAtPoint(x, y, speedModifier);

        if(! con) stopDriving();

        return con;
    }

    /**
     * drive in the direction specified by point while the condition is true.
     * Assumes speed modifier is 1
     * @see Robot9853#driveAtPoint(double, double, double)
     * @see Robot9853#driveAtPointWhile(double, double, double, boolean)
     *
     * @param x     the x part of the direction
     * @param y     the y part of the direction
     * @param con   the condition
     * @return      whether the condition was true
     */
    public boolean driveAtPointWhile(double x, double y, boolean con) {
        return driveAtPointWhile(x, y, 1, con);
    }

    /**
     * drives forward while the condition is true
     * @see Robot9853#driveForward(double)
     *
     * @param speedModifier the speed modified by multiplying it by this number
     * @param con           the condition
     * @return              whether the condition was true
     */
    public boolean driveForwardWhile(double speedModifier, boolean con) {
        driveForward(speedModifier);

        if(! con) stopDriving();

        return con;
    }

    /**
     * drives forward while the condition is true. assumes speed modifier is 1.
     * @see Robot9853#driveForward(double)
     * @see Robot9853#driveForwardWhile(double, boolean)
     *
     * @param con   the condition
     * @return      whether the condition was true
     */
    public boolean driveForWhile(boolean con) {
        return driveForwardWhile(1, con);
    }

    /**
     * Drive in the direction specified by the angle while attempting to reach the target heading
     * while the condition is true.
     * @see Robot9853#driveWithHeading(double, double, int)
     *
     * @param angle         the direction to travel in
     * @param speedModifier the speed modified by multiplying it by this number
     * @param targetHeading the heading to maintain
     * @param con           the condition
     * @return              whether the condition is true
     */
    public boolean driveWithHeadingWhile(double angle, double speedModifier, int targetHeading, boolean con) {
        driveWithHeading(angle, speedModifier, targetHeading);

        if(! con) stopDriving();
        return con;
    }

    /**
     * Drive in the direction specified by the angle while attempting to reach the target heading
     * while  the condition is true. Assumes speed modifier is 1
     * @see Robot9853#driveWithHeading(double, double, int)
     * @see Robot9853#driveWithHeadingWhile(double, double, int, boolean)
     *
     * @param angle         the direction to travel in
     * @param targetHeading the heading to maintain
     * @param con           the condition
     * @return              whether the condition is true
     */
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
     *
     * @param power the power value to set the motors to
     */
    public void setCollectorPower(double power) {
        this.sweeper.setPower(power);
        this.belt.setPower(-power);
    }

    /**
     * Sets just the belts power
     * @see Robot9853#setCollectorPower(double)
     *
     * @param power the power value to set the motors to
     */
    public void setBeltPower(double power) {
        this.belt.setPower(power);
    }

    /**
     * sets the lifts power
     *
     * @param power the power value to set the motor to
     */
    public void setLiftPower(double power) {
        this.lift.setPower(power);
    }

    /**
     * toggle the lift servos position
     */
    public void toggleLift() {
       // if(liftToggle.getPosition() != lastLiftToggle) {
        if(lastLiftToggle == TOGGLE_UP_POSITION) {
            // going up
            liftToggle.setPosition(TOGGLE_DOWN_POSITION);
            lastLiftToggle = TOGGLE_DOWN_POSITION;
        } else {
            // going down
            liftToggle.setPosition(TOGGLE_UP_POSITION);
            lastLiftToggle = TOGGLE_UP_POSITION;
        }
    }
   // }

    /**
     * runs the shooter
     *
     * @param power the power to set the shooter to
     */
    public void setShooterPower(double power){
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shooter.setPower(power);
        isShooterRunning = true;
    }
    public void setShooterPower() {
        setShooterPower(1);
    }

    public int getShooterPosition(){
      return shooter.getCurrentPosition();
    }

    public void setShooterPosition(int pos){
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setTargetPosition(pos);
        isShooterRunning = false;
    }

    public void goToShooterPos0(){
        setShooterPosition(shooter.getCurrentPosition() + (1440-(shooter.getCurrentPosition()%1440)));
    }

    public boolean isShooterRunning(){
        return isShooterRunning;
    }

    /**
     * shoots until the time is up
     * @param power     the power to set the shooter to
     * @param duration  how long to shoot for
     * @return          whether time is up
     */
    public boolean shootFor(double power, long duration) {
        boolean result = doUntil(duration);

        setShooterPower(power);

        if(isTimerUp()) {
            goToShooterPos0();
        }

        return result;
    }
    public boolean shootFor(long duration) {
        return shootFor(1, duration);
    }

    /**
     * runs the collector until time is up
     *
     * @param duration   the time to end
     * @return          whether time is up
     */
    public boolean reloadFor(long duration) {
        boolean result = doUntil(duration);

        setCollectorPower(1);

        if(isTimerUp()) {
            setCollectorPower(0);
        }

        return result;
    }

    /**
     * Wait for the gyro to finish calibrating
     */
    public void waitForGyroCalibration() {
        while(gyro.isCalibrating()) {
            // stall
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
     * whether the beacon is in range
     * @return whether beacon is readable
     */
    public boolean isBeaconInRange() {
        if(centerBeaconSensor.getColorNumber() == 3 || centerBeaconSensor.getColorNumber() == 10)
            return true;

        if(rightBeaconSensor.getColorNumber() == 3 || rightBeaconSensor.getColorNumber() == 10)
            return true;

        return false;
    }

    /**
     * determine whether or not the beacon is red
     * @return whether or not the beacon sensor is reading red
     */
    public boolean isBeaconRed() {
        return centerBeaconSensor.getColorNumber() == 10;
    }

    /**
     * determine whether or not the beacon is blue
     * @return whether or not the beacon sensor is reading red
     */
    public boolean isBeaconBlue() {
        return centerBeaconSensor.getColorNumber() == 3;
    }

    /**
     * Get the right color of a beacon
     *
     * @return the color number
     */
    public int getRightBeaconColor() {
        return rightBeaconSensor.getColorNumber();
    }

    /**
     * Get the left color of a beacons
     *
     * @return the color number
     */
    public int getLeftBeaconColor() {
        return centerBeaconSensor.getColorNumber();
    }

    public boolean isBlue(int colorNumber) {
        return colorNumber <= 4;
    }

    public boolean isRed(int colorNumber) {
        return colorNumber >= 9;
    }
}

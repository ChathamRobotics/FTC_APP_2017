package org.firstinspires.ftc.team11248;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.chathamrobotics.ftcutils.MRColorSensorV2;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Robot Class for team 11248
 */

public class Robot11248 extends OmniWheelDriver {

    //Debouncing Variables


    private boolean shooterOn, conveyorOn;

    //Angles
    public static final double RIGHT_ANGLE = Math.PI/2;

    //Driving constants
//    public static final double MAX_TURN = .30;
//    public static final double MAX_SPEED = .70;
    public static final double SHOOTER_SPEED = 1;

    //GYRO ROTATION
    private static final double GYRO_ROTATION_BLUE = -.25;
    private static final int DEGREE_THRESHOLD = 4;

    //LINE SENSOR THRESHOLDS
    private static final double OPTICAL_THRESHOLD_LOW = .9;
    private static final double OPTICAL_THRESHOLD_HIGH = 1;

    //Servo constants
    private static final double LIFT_UP = .30;
    private static final double LIFT_DOWN = 1;

    private static final double BEACON_OUT = 0;
    private static final double BEACON_IN = 1;

    //Beacon Setup
    MRColorSensorV3 colorBeacon;
    OpticalDistanceSensor lineSensor;

    // I2C address, registers, and commands
    private final byte COLOR_SENSOR_GREEN_ADDR = 0x3A; //Green
    private final byte COLOR_SENSOR_YELLOW_ADDR = 0x3C; //Yellow
    private final byte COLOR_SENSOR_BEACON_ADDR = 0x3E; //Beacon

    private final int BLUE_LOW_THRESHOLD = 2;
    private final int BLUE_HIGH_THRESHOLD = 3;
    private final int RED_LOW_THRESHOLD = 10;
    private final int RED_HIGH_THRESHOLD = 11;
    private final int LINE_LOW_THRESHOLD = 15;
    private final int LINE_HIGH_THRESHOLD = 16;

    //Motors, Sensors, Telemetry
    private DcMotor shooterL, shooterR, lift, conveyor;
    private Servo liftArm, beaconPusher;
    private boolean isLiftArmUp = false;

    //hardware map
    public static final String[] MOTOR_LIST =
            {"FrontLeft","FrontRight","BackLeft","BackRight",
                    "ShooterL","ShooterR","Lift","Conveyor"};

    public static final String[] SERVO_LIST =
            {"servo1", "servo2"};

    public static final String COLOR = "color2";

    public static final String GYRO = "gyro";

    public static final String LINE = "sensor_ods";

    /**
     * Initializes using a list of motors.
     * @param motors
     * @param servos
     * @param color
     * @param gyro
     * @param line
     * @param telemetry
     */
    public Robot11248(DcMotor[] motors, Servo[] servos, I2cDevice color, GyroSensor gyro, OpticalDistanceSensor line, Telemetry telemetry) {
        this(motors[0], motors[1], motors[2], motors[3], motors[4], motors[5], motors[6],
                motors[7], servos[0], servos[1], color, gyro, line, telemetry);
    }

    /**
     * Creates a model of the robot and initializes sensors, motors, and telemetry
     * @param frontLeft - wheel motor
     * @param frontRight - wheel motor
     * @param backLeft - wheel motor
     * @param backRight - wheel motor
     * @param shooterL - shooter motor
     * @param shooterR - shooter motor
     * @param conveyor - conveyor motor
     * @param lift - lift motor
     * @param liftArm - lift release servo
     * @param beaconPusher - beacon pusher servo
     * @param colorBeacon - color sensor for beacons 0x3E
     * @param gyro - gyroscopic sensor
     * @param line - optical distance sensor for the line
     * @param telemetry
     */
    public Robot11248(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight,
                      DcMotor shooterL, DcMotor shooterR, DcMotor lift, DcMotor conveyor,
                      Servo liftArm, Servo beaconPusher, I2cDevice colorBeacon, GyroSensor gyro,
                      OpticalDistanceSensor line, Telemetry telemetry) {

        super(frontLeft, frontRight, backLeft, backRight, gyro, telemetry);
        this.shooterL = shooterL;
        this.shooterR = shooterR;
        this.lift = lift;
        this.liftArm = liftArm;
        this.conveyor = conveyor;
        this.beaconPusher = beaconPusher;
        this.lineSensor = line;

        this.colorBeacon = new MRColorSensorV3(colorBeacon, COLOR_SENSOR_BEACON_ADDR);

        //this.shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //this.shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Initializes the robot. (In this case it just sets servo positions to default)
     */
    public void init() {
        moveLiftArmUp();
        moveBeaconIn();

    }

    public void moveLiftArmUp() {
        liftArm.setPosition(LIFT_UP);
        isLiftArmUp = true;
    }

    public void moveLiftArmDown() {
        liftArm.setPosition(LIFT_DOWN);
        isLiftArmUp = false;
    }

    public void switchLiftArm() {
        if(isLiftArmUp)
            moveLiftArmDown();
        else
            moveLiftArmUp();
    }

    public void moveBeaconOut(){
        beaconPusher.setPosition(BEACON_OUT);
    }

    public void moveBeaconIn(){
        beaconPusher.setPosition(BEACON_IN);
    }

    public void setShooter(double SHOOTER_SPEED) {
        shooterL.setPower(SHOOTER_SPEED);
        shooterR.setPower(-SHOOTER_SPEED);
        shooterOn = true;
        if(SHOOTER_SPEED == 0)
            shooterOn = false;
    }

    public void shooterReverse() {
        shooterL.setPower(SHOOTER_SPEED);
        shooterR.setPower(-SHOOTER_SPEED);
        shooterOn = true;
    }

    //Overload
    public void shooterOn() {
        shooterL.setPower(SHOOTER_SPEED);
        shooterR.setPower(-SHOOTER_SPEED);
        shooterOn = true;
    }

    public void shooterOff() {
        shooterL.setPower(0);
        shooterR.setPower(0);
        shooterOn = false;
    }

    public void setLiftSpeed(double speed) {
        if(speed > 1)
            speed = 1;
        if(speed < -1)
            speed = -1;
        lift.setPower(speed);
    }

    public void setConveyor(float f){
        conveyor.setPower(f);
        conveyorOn = true;
        if(f == 0)
            conveyorOn = false;
    }

    public void conveyorOn(){
        conveyor.setPower(1);
        conveyorOn = true;
    }

    public void conveyorReverse(){
        conveyor.setPower(-1);
        conveyorOn = true;
    }

    public void conveyorOff(){
        conveyor.setPower(0);
        conveyorOn = false;
    }

    public boolean getConveyerOn() {
        return conveyorOn;
    }

    public boolean getShooterOn() {
        return shooterOn;
    }

    //BEACONS

    public void activateColorSensors(){
        colorBeacon.setPassiveMode();
    }

    public void deactivateColorSensors(){
        colorBeacon.setPassiveMode();
    }


    public int getColorBeacon(){
        return colorBeacon.getColorNumber();
    }

    public boolean isBeaconBlue(){
        int color = colorBeacon.getColorNumber();
        return (BLUE_LOW_THRESHOLD <= color && color <= BLUE_HIGH_THRESHOLD);
    }

    public boolean isBeaconRed(){
        int color = colorBeacon.getColorNumber();
        return (RED_LOW_THRESHOLD <= color && color <= RED_HIGH_THRESHOLD);
    }

    public boolean hitLine(){
        return (lineSensor.getLightDetected() < OPTICAL_THRESHOLD_HIGH &&
                lineSensor.getLightDetected() >= OPTICAL_THRESHOLD_LOW);
    }

    public void driveWithGyro2(double x, double y, int targetAngle){
        getTelemetry().addData("angle", getGyroAngle());
        getTelemetry().addData("angle2", targetAngle);

        if(angleWithinThreshold(getGyroAngle(),targetAngle))
            driveold(x,y,0,false);
        else
            driveold(x,y,GYRO_ROTATION_BLUE,false);
    }

    public static boolean angleWithinThreshold(double current, double target) {
        return Math.abs(current - target) <= DEGREE_THRESHOLD;
    }
}

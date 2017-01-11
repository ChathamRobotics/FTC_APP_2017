package org.firstinspires.ftc.team11248;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import org.chathamrobotics.ftcutils.MRColorSensorV3;
import org.chathamrobotics.ftcutils.OmniWheelDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Robot Class for team 11248
 */

public class Robot11248 extends OmniWheelDriver {

    //Debouncing Variables


    public boolean shooterOn, conveyorOn, collectorClosed, isLiftArmUp;

    //Angles
    public static final double RIGHT_ANGLE = Math.PI/2;

    //Driving constants
    public static final double SHOOTER_SPEED = .8;

    //GYRO ROTATION
    private static final double GYRO_ROTATION_BLUE = -.25;
    private static final int GYRO_THRESHOLD = 2;

    //LINE SENSOR THRESHOLDS
    private static final double OPTICAL_THRESHOLD_LOW = .6;
    private static final double OPTICAL_THRESHOLD_HIGH = 1;

    //Servo constants
    private static final double LIFT_DOWN = .26;
    private static final double LIFT_UP = 1;

    private static final double BEACON_OUT = 0;
    private static final double BEACON_IN = 1;

    private static final double COLLECTOR_L_OPEN = .85;
    private static final double COLLECTOR_L_CLOSED = .425;
    private static final double COLLECTOR_R_OPEN = .05;
    private static final double COLLECTOR_R_CLOSED = .425;

    // I2C address, registers, and commands
    private final byte COLOR_SENSOR_GREEN_ADDR = 0x3A; //Green
    private final byte COLOR_SENSOR_YELLOW_ADDR = 0x3C; //Yellow
    private final byte COLOR_SENSOR_BEACON_ADDR = 0x3E; //Beacon


    //Color sensor color thresholds
    private final int BLUE_LOW_THRESHOLD = 1;
    private final int BLUE_HIGH_THRESHOLD = 4;
    private final int RED_LOW_THRESHOLD = 10;
    private final int RED_HIGH_THRESHOLD = 11;
    private final int LINE_LOW_THRESHOLD = 15;
    private final int LINE_HIGH_THRESHOLD = 16;

    //Motors, Sensors, Telemetry
    private DcMotor shooterL, shooterR, lift, conveyor;
    private Servo liftArm, beaconPusher, collectorServoL, collectorServoR;
    private MRColorSensorV3 colorBeacon;
    private OpticalDistanceSensor lineSensor;
    private GyroSensor gyro;
    private UltrasonicSensor sonar;
    private Telemetry telemetry;

    //hardware map
    public static final String[] MOTOR_LIST =
            {"FrontLeft","FrontRight","BackLeft","BackRight",
                    "ShooterL","ShooterR","Lift","Conveyor"};

    public static final String[] SERVO_LIST =
            {"servo1", "servo2","servo3","servo4"};

    public static final String COLOR = "color2";

    public static final String GYRO = "gyro";

    public static final String LINE = "sensor_ods";

    public static final String SONAR = "sonar";

    /**
     * Initializes using a list of motors.
     * @param motors
     * @param servos
     * @param color
     * @param gyro
     * @param line
     * @param sonar
     * @param telemetry
     */
    public Robot11248(DcMotor[] motors, Servo[] servos, I2cDevice color, GyroSensor gyro, OpticalDistanceSensor line, UltrasonicSensor sonar, Telemetry telemetry) {
        this(motors[0], motors[1], motors[2], motors[3], motors[4], motors[5], motors[6],
                motors[7], servos[0], servos[1], servos[2], servos[3], color, gyro, line, sonar,telemetry);
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
     * @param collectorServoL - servo for collecting on the left side
     * @param collectorServoR - servo for collecting on the right side
     * @param colorBeacon - color sensor for beacons 0x3E
     * @param gyro - gyroscopic sensor
     * @param line - optical distance sensor for the line
     * @param sonar - ultrasonic sensor through legacy module
     * @param telemetry
     */
    public Robot11248(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight,
                      DcMotor shooterL, DcMotor shooterR, DcMotor lift, DcMotor conveyor,
                      Servo liftArm, Servo beaconPusher, Servo collectorServoL, Servo collectorServoR, I2cDevice colorBeacon, GyroSensor gyro,
                      OpticalDistanceSensor line, UltrasonicSensor sonar,Telemetry telemetry) {

        super(frontLeft, frontRight, backLeft, backRight, telemetry);
        this.shooterL = shooterL;
        this.shooterR = shooterR;
        this.lift = lift;
        this.liftArm = liftArm;
        this.conveyor = conveyor;
        this.beaconPusher = beaconPusher;
        this.collectorServoL = collectorServoL;
        this.collectorServoR = collectorServoR;
        this.lineSensor = line;
        this.gyro = gyro;
        this.sonar = sonar;

        this.colorBeacon = new MRColorSensorV3(colorBeacon, COLOR_SENSOR_BEACON_ADDR);

            //this.shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //this.shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Initializes the robot. (In this case it just sets servo positions to default)
     */
    public void init() {
        moveLiftArmDown();
        moveBeaconIn();
        closeCollector();

    }

    /*
     * TELEMETRY METHODS
     */

    public void showMotorValues(){
    }


     /*
     * BEACON PUSHER SERVO METHODS
     */

    public void moveBeaconOut(){
        beaconPusher.setPosition(BEACON_OUT);
    }

    public void moveBeaconIn(){
        beaconPusher.setPosition(BEACON_IN);
    }


    /*
    * LIFT METHODS
    */
    public void setLiftSpeed(double speed) {
        if(speed > 1)
            speed = 1;
        if(speed < -1)
            speed = -1;
        lift.setPower(speed);
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


    /*
    * SHOOTER METHODS
    */

    public void setShooter(double SHOOTER_SPEED) {
        shooterL.setPower(SHOOTER_SPEED);
        shooterR.setPower(-SHOOTER_SPEED);
        shooterOn = true;
        if(SHOOTER_SPEED == 0)
            shooterOn = false;
    }

    public void shooterReverse() {
        shooterL.setPower(-SHOOTER_SPEED);
        shooterR.setPower(SHOOTER_SPEED);
        shooterOn = true;
    }

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

    public boolean getShooterOn() {
        return shooterOn;
    }


     /*
     * COLLECTOR METHODS
     */

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

    public void openCollector(){
        collectorServoL.setPosition(COLLECTOR_L_OPEN);
        collectorServoR.setPosition(COLLECTOR_R_OPEN);
    }

    public void closeCollector(){
        collectorServoL.setPosition(COLLECTOR_L_CLOSED);
        collectorServoR.setPosition(COLLECTOR_R_CLOSED);
    }

    public void switchCollectorServo() {
        if (collectorClosed){
            openCollector();
        collectorClosed = false;
    }else{
            closeCollector();
            collectorClosed = true;
        }
    }



    /*
     * COLOR SENSOR METHODS
     */

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


    /*
     * OPTICAL DISTANCE SENSOR METHODS
     */

    public boolean hitLine(){
        return (lineSensor.getLightDetected() < OPTICAL_THRESHOLD_HIGH &&
                lineSensor.getLightDetected() >= OPTICAL_THRESHOLD_LOW);
    }

    public double getLineSensorValue(){
        return lineSensor.getLightDetected();
    }


    /*
     * GYRO SENSOR METHODS
     */

    public void calibrateGyro(){
        gyro.calibrate();
        while(gyro.isCalibrating());
    }

    public int getGyroAngle(){
        return gyro.getHeading();
    }

    public boolean driveWithGyro(double x, double y, int targetAngle) {

        boolean atAngle = false;
        int currentAngle = getGyroAngle();
        int net = currentAngle - targetAngle; //finds distance to target angle
        double rotation;

        if (Math.abs(net) > 180) { // if shortest path passes 0
            if (currentAngle > 180) //if going counterclockwise
                net = (currentAngle - 360) - targetAngle;

            else //if going clockwise
                net = (360 - targetAngle) + currentAngle;
        }

        // slows down as approaches angle with min threshold of .05
        // each degree adds/subtracts .95/180 values of speed
        rotation = Math.abs(net) * .85 / 180 + .15;

        if (net < 0) rotation *= -1; //if going clockwise, set rotation clockwise (-)

        if (Math.abs(net) > GYRO_THRESHOLD)
            driveold(x, y, rotation); //Drive with gyros rotation

        else {
            atAngle = true;
            driveold(x, y, 0);
        }

        if(silent) {
            telemetry.addData("ROBOT11248", "Heading: " + getGyroAngle());
            telemetry.addData("ROBOT11248", "Net: " + net);
            telemetry.addData("ROBOT11248", "Speed: " + rotation);
            telemetry.addData("ROBOT11248", "Target: " + targetAngle);
            telemetry.update();
        }

        return atAngle;
    }

    public boolean moveToAngle(int targetAngle){
        return driveWithGyro(0,0,targetAngle);
    }


    /*
     * SONAR SENSOR METHODS
     */

    public double lastVal = 255;
    public double getSonarValue(){


        return Range.clip(sonar.getUltrasonicLevel(), 0, 255);

       // double val = sonar.getUltrasonicLevel();
        // if(val != 0  && val != 255 ) {
//        val = lastVal;
//        return val;

       // }else
            //return lastVal;

    }

    public double getDistanceIN(){
        //TODO: Calibrate
        int unitsPerIn = 1023;
        return sonar.getUltrasonicLevel()/unitsPerIn;
    }

    public double getDistanceCM(){
        //TODO: Calibrate
        int unitsPerCm = 1023;
        return sonar.getUltrasonicLevel()/unitsPerCm;
    }

    public double getDistanceMM(){
        //TODO: Calibrate
        int unitsPerMm = 1023;
        return sonar.getUltrasonicLevel()/unitsPerMm;
    }


}

package org.firstinspires.ftc.team11248;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;

import org.chathamrobotics.ftcutils.hardware.MRColorSensorV3;
import org.chathamrobotics.ftcutils.OmniWheelDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team11248.testModes.EncoderTest;
import org.firstinspires.ftc.team11248.testModes.Thread_Test;

import java.util.Timer;
import java.util.TimerTask;

/**
 * Robot Class for team 11248
 */

public class Robot11248 extends OmniWheelDriver {

    //Debouncing Variables
    public boolean shooterOn, conveyorOn, collectorClosed, isLiftArmUp;

    public static boolean bangBangOn;

    //Angles
    public static final double RIGHT_ANGLE = Math.PI/2;

    //Driving constants
    public static final double SHOOTER_SPEED = 1f;
    public static final long SHOOTER_RPM = 50;
    public static final int LOOP = 10;
    public static final double AUTO_SHOOTER_SPEED = .275f;

    //Gyro Thresholds
    private static final int GYRO_THRESHOLD = 1;

    //LINE SENSOR THRESHOLDS
    private static final double OPTICAL_THRESHOLD_LOW = .6;
    private static final double OPTICAL_THRESHOLD_HIGH = 1;
    private static final double OPTICAL_THRESHOLD2_LOW = .6;
    private static final double OPTICAL_THRESHOLD2_HIGH = 1;

    //Servo constants
    private static final double LIFT_DOWN = .26;
    private static final double LIFT_UP = 1;

    private static final double BEACON_OUT = 0;
    private static final double BEACON_IN = 1;

    private static final double COLLECTOR_L_OPEN = 0;
    private static final double COLLECTOR_L_CLOSED = .4;
    private static final double COLLECTOR_R_OPEN = .7;
    private static final double COLLECTOR_R_CLOSED = .3;

    // I2C address, registers, and commands
    private final byte COLOR_SENSOR_GREEN_ADDR = 0x3A; //Green
    private final byte COLOR_SENSOR_YELLOW_ADDR = 0x3C; //Yellow
    private final byte COLOR_SENSOR_BEACON_ADDR = 0x3E; //Beacon


    //Color sensor color thresholds
    private final int BLUE_LOW_THRESHOLD = 1;
    private final int BLUE_HIGH_THRESHOLD = 4;
    private final int RED_LOW_THRESHOLD = 10;
    private final int RED_HIGH_THRESHOLD = 11;


    //Motors, Sensors, Telemetry
    public static volatile DcMotor shooterL, shooterR;
    private DcMotor lift, conveyor;
    private Servo liftArm, beaconPusher, collectorServoL, collectorServoR;
    private ServoController servoController;
    private MRColorSensorV3 colorBeacon;
    private OpticalDistanceSensor lineSensor;
    private OpticalDistanceSensor lineSensor2;
    private GyroSensor gyro;
    private UltrasonicSensor sonar;
    private DeviceInterfaceModule dim;
    public static Telemetry telemetry;


    public Robot11248(HardwareMap hardwareMap, Telemetry telemetry){

        /*
         * MOTOR INITS
         */
        super(hardwareMap.dcMotor.get("FrontLeft"),
                hardwareMap.dcMotor.get("FrontRight"),
                hardwareMap.dcMotor.get("BackLeft"),
                hardwareMap.dcMotor.get("BackRight"),
                telemetry);

        Robot11248.shooterL = hardwareMap.dcMotor.get("ShooterL");
        Robot11248.shooterR = hardwareMap.dcMotor.get("ShooterR");
        this.lift = hardwareMap.dcMotor.get("Lift");
        this.conveyor = hardwareMap.dcMotor.get("Conveyor");


        Robot11248.shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot11248.shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot11248.shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot11248.shooterR.setDirection(DcMotorSimple.Direction.REVERSE);


        Robot11248.shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot11248.shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot11248.shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
         * SERVO INITS
         */
        this.liftArm = hardwareMap.servo.get("servo1");
        this.beaconPusher = hardwareMap.servo.get("servo2");
        this.collectorServoL = hardwareMap.servo.get("servo3");
        this.collectorServoR = hardwareMap.servo.get("servo4");
        this.servoController = hardwareMap.servoController.get("Servo Controller 3");

        /*
         * SENSOR INITS
         */
        this.lineSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");
        this.lineSensor2 = hardwareMap.opticalDistanceSensor.get("sensor_ods2");
        this.gyro = hardwareMap.gyroSensor.get("gyro");
        this.sonar = hardwareMap.ultrasonicSensor.get("sonar");
        I2cDevice colorBeacon = hardwareMap.i2cDevice.get("color2");
        this.colorBeacon = new MRColorSensorV3(colorBeacon, COLOR_SENSOR_BEACON_ADDR);

        //TELEMETRY
        Robot11248.telemetry = telemetry;
        this.dim = hardwareMap.get(DeviceInterfaceModule.class, "Device Interface Module 1");

    }

    /**
     * Initializes the robot. (In this case it just sets servo positions to default)
     */
    public void init() {
        moveBeaconIn();
        moveLiftArmDown();
        closeCollector();
        setDimLed(true,true);

//        Thread bangBangLeft = new BangBangLeft();
//        Thread bangBangRight = new BangBangRight();
//        new Thread(bangBangLeft).start();
//        new Thread(bangBangRight).start();


    }

    public void activateServos(){
        servoController.pwmEnable();
    }
    public void deactivateServos(){
        servoController.pwmDisable();
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
        shooterR.setPower(SHOOTER_SPEED);
        shooterOn = (SHOOTER_SPEED!=0);
        if(shooterOn)openCollector();
    }



    public void shooterReverse() {
        shooterL.setPower(-SHOOTER_SPEED);
        shooterR.setPower(-SHOOTER_SPEED);
        openCollector();
        bangBangOn = false;
        shooterOn = true;
    }

    public void shooterOn() {
        shooterL.setPower(SHOOTER_SPEED);
        shooterR.setPower(SHOOTER_SPEED);
        openCollector();
        bangBangOn = false;
        shooterOn = true;
    }

    public void shooterOff() {
        bangBangOn = false;
        shooterOn = false;
        shooterL.setPower(0);
        shooterR.setPower(0);
        closeCollector();
    }

    public void shooterOnBang() {
        openCollector();
        shooterOn = true;
        bangBangOn = true;
    }


    public boolean getShooterOn() {
        return shooterOn;
    }

     /*
     * COLLECTOR METHODS
     */
    public void setConveyor(float f){
        conveyor.setPower(f);
        conveyorOn = (f!= 0);

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
        collectorClosed = false;
    }

    public void closeCollector(){
        collectorServoL.setPosition(COLLECTOR_L_CLOSED);
        collectorServoR.setPosition(COLLECTOR_R_CLOSED);
        collectorClosed = true;
    }

    public void switchCollectorServo() {
        if (collectorClosed){
            openCollector();
        }else{
            closeCollector();
            shooterOff();
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

    /*
     * OPTICAL DISTANCE SENSOR METHODS
     */
    public boolean hitLine2(){
        return (lineSensor2.getLightDetected() < OPTICAL_THRESHOLD2_HIGH &&
                lineSensor2.getLightDetected() >= OPTICAL_THRESHOLD2_LOW);
    }


    public double getLineSensorValue(){
        return lineSensor.getLightDetected();
    }

    public double getLineSensor2Value(){
        return lineSensor2.getLightDetected();
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

    public boolean driveWithGyro(double x, double y, int targetAngle, boolean smooth) {

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
        rotation = Math.abs(net) * .85 / 180 + .10;

        if (net < 0) rotation *= -1; //if going clockwise, set rotation clockwise (-)

        if (!(Math.abs(net) > GYRO_THRESHOLD)){
            atAngle = true;
            rotation = 0;
        }

        driveWithFixedAngle(x, y, rotation, 360 - getGyroAngle() + targetAngle, smooth); //Drive with gyros rotation

        if(silent) {
            telemetry.addData("ROBOT11248", "Heading: " + getGyroAngle());
            telemetry.addData("ROBOT11248", "Net: " + net);
            telemetry.addData("ROBOT11248", "Speed: " + rotation);
            telemetry.addData("ROBOT11248", "Target: " + targetAngle);
            telemetry.update();
        }

        return atAngle;
    }


    public boolean driveWithGyro(double x, double y, int targetAngle) {
        return driveWithGyro(x,y,targetAngle, false);
    }


    /**
     *
     * @param x - x direction power
     * @param y - y direction power
     * @param rotate - power for rotation
     * @param fixedAngle -  angle orentation is fixed on - set = to 359 - getGyroAngle()
     */
    public void driveWithFixedAngle(double x, double y, double rotate, int fixedAngle, boolean smooth){

        setOffsetAngle((Math.toRadians(fixedAngle)));
        driveold(x, y, rotate, smooth);
    }

    public void driveWithFixedAngle(double x, double y, double rotate, int fixedAngle) {

        driveWithFixedAngle(x, y, rotate, fixedAngle, false);
    }


    public boolean moveToAngle(int targetAngle){
        return driveWithGyro(0,0,targetAngle);
    }

    /*
     * SONAR SENSOR METHODS
     */
    public double getSonarValue(){
        return Range.clip(sonar.getUltrasonicLevel(), 0, 255);

    }

    /*
     * LED METHODS
     */
    public void setDimLed(boolean red, boolean blue){
        dim.setLED(0, red);
        dim.setLED(1, blue);
    }

}


class BangBangLeft extends Thread{

    int lastEncoder, currentEncoder;
    double rpm;
    long loopSpeed = Robot11248.LOOP;

    BangBangLeft (){

        currentEncoder = lastEncoder = Robot11248.shooterL.getCurrentPosition();
    }

    private void getSpeed(){

        lastEncoder = currentEncoder;
        currentEncoder = Robot11248.shooterL.getCurrentPosition();
        rpm = ((currentEncoder-lastEncoder)/1440.0)/(loopSpeed * .001*.0166666667);
    }

    @Override
    public void run() {

        while (!isInterrupted()) {

            getSpeed();

            if (Robot11248.bangBangOn) {

                if (rpm >= Robot11248.SHOOTER_RPM) {
                    Robot11248.shooterL.setPower(.2);

                } else if (rpm < Robot11248.SHOOTER_RPM) {
                    Robot11248.shooterL.setPower(.8);
                }

            }

            try {
                Thread.sleep(loopSpeed);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}

class BangBangRight extends Thread{

    int lastEncoder, currentEncoder;
    double rpm;
    long loopSpeed = Robot11248.LOOP;

    BangBangRight (){

        currentEncoder = lastEncoder = Robot11248.shooterR.getCurrentPosition();
    }

    private void getSpeed(){

        lastEncoder = currentEncoder;
        currentEncoder = Robot11248.shooterR.getCurrentPosition();
        rpm = ((currentEncoder-lastEncoder)/1440.0)/(loopSpeed * .001*.0166666667);
    }

    @Override
    public void run() {

        while (!isInterrupted()) {

            getSpeed();

            if (Robot11248.bangBangOn) {

                if (rpm >= Robot11248.SHOOTER_RPM) {
                    Robot11248.shooterR.setPower(.2);

                } else if (rpm < Robot11248.SHOOTER_RPM) {
                    Robot11248.shooterR.setPower(.8);
                }

            }

            try {
                Thread.sleep(loopSpeed);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
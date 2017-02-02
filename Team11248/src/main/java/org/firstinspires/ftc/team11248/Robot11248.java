package org.firstinspires.ftc.team11248;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import org.chathamrobotics.ftcutils.hardware.MRColorSensorV3;
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

    //Gyro Thresholds
    private static final int GYRO_THRESHOLD = 2;

    //LINE SENSOR THRESHOLDS
    private static final double OPTICAL_THRESHOLD_LOW = .59;
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


    //Motors, Sensors, Telemetry
    public DcMotor shooterL, shooterR, lift, conveyor;
    private Servo liftArm, beaconPusher, collectorServoL, collectorServoR;
    private MRColorSensorV3 colorBeacon;
    private OpticalDistanceSensor lineSensor;
    private GyroSensor gyro;
    private UltrasonicSensor sonar;
    private DeviceInterfaceModule dim;
    private Telemetry telemetry;

    //private DeviceInterfaceModule dim;


    public Robot11248(HardwareMap hardwareMap, Telemetry telemetry){

        /*
         * MOTOR INITS
         */
        super(hardwareMap.dcMotor.get("FrontLeft"),
                hardwareMap.dcMotor.get("FrontRight"),
                hardwareMap.dcMotor.get("BackLeft"),
                hardwareMap.dcMotor.get("BackRight"),
                telemetry);

        this.shooterL = hardwareMap.dcMotor.get("ShooterL");
        this.shooterR = hardwareMap.dcMotor.get("ShooterR");
        this.lift = hardwareMap.dcMotor.get("Lift");
        this.conveyor = hardwareMap.dcMotor.get("Conveyor");


        this.shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        this.shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        this.shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
         * SERVO INITS
         */
        this.liftArm = hardwareMap.servo.get("servo1");
        this.beaconPusher = hardwareMap.servo.get("servo2");
        this.collectorServoL = hardwareMap.servo.get("servo3");
        this.collectorServoR = hardwareMap.servo.get("servo4");

        /*
         * SENSOR INITS
         */
        this.lineSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");
        this.gyro = hardwareMap.gyroSensor.get("gyro");
        this.sonar = hardwareMap.ultrasonicSensor.get("sonar");

        I2cDevice colorBeacon = hardwareMap.i2cDevice.get("color2");
        this.colorBeacon = new MRColorSensorV3(colorBeacon, COLOR_SENSOR_BEACON_ADDR);

        //TELEMETRY
        this.telemetry = telemetry;
        this.dim = hardwareMap.get(DeviceInterfaceModule.class, "Device Interface Module 1");

    }

    /**
     * Initializes the robot. (In this case it just sets servo positions to default)
     */
    public void init() {
        moveLiftArmDown();
        moveBeaconIn();
        closeCollector();
        setDimLed(true,true);

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




    long lastTime = System.nanoTime();
    int leftLastEncoder = 0;
    int rightLastEncoder = 0;
    double tolerance = .015;

    public void bangBang(double fTarget){

        boolean up = false;
        long now = System.nanoTime();
        long elapsedTime = now - lastTime;
        double left = fTarget;
        double right = fTarget;

        int shooterL_Encoder = shooterL.getCurrentPosition();
        int shooterR_Encoder = shooterR.getCurrentPosition();

        double shooterL_Velocity = (double)(shooterL_Encoder - leftLastEncoder) / elapsedTime;
        double shooterR_Velocity = (double)(shooterR_Encoder - rightLastEncoder) / elapsedTime;


        if(shooterL_Velocity >= (fTarget + tolerance)){
            left = fTarget - .05;
            up = true;

        } else if(shooterL_Velocity < (fTarget - tolerance)) {
            left = fTarget + .05;
        }


        if(shooterR_Velocity >= (fTarget + tolerance)){
            right = fTarget - .05;

        } else if(shooterR_Velocity < (fTarget - tolerance)) {
            right = fTarget + .05;
        }


        shooterL.setPower(Range.clip(left, 0, 1));
        shooterR.setPower(-Range.clip(right,0 ,1));
        shooterOn = true;

        leftLastEncoder = shooterL_Encoder;
        rightLastEncoder = shooterR_Encoder;

        lastTime = now;

        telemetry.addData("1", up);
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
    public double getSonarValue(){
        return Range.clip(sonar.getUltrasonicLevel(), 0, 255);

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

    /*
     * LED METHODS
     */
    public void setDimLed(boolean red, boolean blue){
        dim.setLED(0, red);
        dim.setLED(1, blue);
    }

}
package org.firstinspires.ftc.team11248;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.chathamrobotics.ftcutils.OmniWheelDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Robot Class for team 11248
 */

public class Robot11248 extends OmniWheelDriver {

    //Angles
    public static final double RIGHT_ANGLE = Math.PI/2;

    //Driving constants
    public static final double MAX_TURN = .30;
    public static final double MAX_SPEED = .70;
    public static final double CONVEYOR_SPEED = .9;

    //Servo constants
    public static final double PADDLE_DOWN = .45;
    public static final double PADDLE_UP = .85;
    public static final double LIFT_UP = 0;
    public static final double LIFT_DOWN = 1;

    //Motors, Sensors, Telemetry
    private DcMotor shooterL, shooterR, lift;
    private Servo paddle,liftArm;
    private Telemetry telemetry;
    private boolean isLiftArmUp = false;
    private boolean isPaddleUp = false;

    //hardware map
    public static final String[] MOTOR_LIST =
            {"FrontLeft","FrontRight","BackLeft","BackRight","ShooterL","ShooterR","Lift"};

    public static final String[] SERVO_LIST =
            {"servo1","servo2"};

    /**
     * Initializes using a list of motors.
     * @param motors
     * @param servos
     * @param telemetry
     */
    public Robot11248(DcMotor[] motors, Servo[] servos, Telemetry telemetry) {
        this(motors[0],motors[1],motors[2],motors[3],motors[4],motors[5],
                motors[6],servos[0],servos[1],telemetry);
    }

    /**
     * Creates a model of the robot and initializes sensors, motors, and telemetry
     * @param frontLeft - wheel motor
     * @param frontRight - wheel motor
     * @param backLeft - wheel motor
     * @param backRight - wheel motor
     * @param shooterL - shooter motor
     * @param shooterR - shooter motor
     * @param lift - lift motor
     * @param paddle - shooter paddle servo
     * @param liftArm - lift release servo
     * @param telemetry
     */
    public Robot11248(DcMotor frontLeft,DcMotor frontRight,DcMotor backLeft,DcMotor backRight,
                      DcMotor shooterL,DcMotor shooterR,DcMotor lift,
                      Servo paddle, Servo liftArm, Telemetry telemetry) {
        super(frontLeft, frontRight, backLeft, backRight, telemetry);
        this.shooterL = shooterL;
        this.shooterR = shooterR;
        this.lift = lift;
        this.paddle = paddle;
        this.liftArm = liftArm;
    }

    /**
     * Initializes the robot. (In this case it just sets servo positions to default)
     */
    public void init() {
        moveLiftArmUp();
        movePaddleDown();
    }

    /**
     * Controls shooter paddle
     */
    public void movePaddleUp() {
        paddle.setPosition(PADDLE_UP);
        isPaddleUp = true;
    }

    /**
     * Controls shooter paddle
     */
    public void movePaddleDown() {
        paddle.setPosition(PADDLE_DOWN);
        isPaddleUp = false;
    }

    public void moveLiftArmUp() {
        liftArm.setPosition(LIFT_UP);
        isLiftArmUp = true;
    }

    public void moveLiftArmDown() {
        liftArm.setPosition(LIFT_DOWN);
        isLiftArmUp = false;
    }

    public void switchPaddle() {
        if(isPaddleUp)
            movePaddleDown();
        else
            movePaddleUp();
    }

    public void switchLiftArm() {
        if(isLiftArmUp)
            moveLiftArmDown();
        else
            moveLiftArmUp();
    }

    public void shooterOn() {
        shooterL.setPower(CONVEYOR_SPEED);
        shooterR.setPower(-CONVEYOR_SPEED);
    }

    public void shooterOff() {
        shooterL.setPower(0);
        shooterR.setPower(0);
    }

    public void setLiftSpeed(double speed) {
        if(speed > 1)
            speed = 1;
        if(speed < -1)
            speed = -1;
        lift.setPower(speed);
    }
}

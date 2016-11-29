package org.chathamrobotics.ftcutils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Driving with omni wheels
 */
public class OmniWheelDriver {
    // Constants
    public static final double OMNI_WHEEL_ANGLE_CORRECTION = Math.PI/4;
    public static final double FRONT_OFFSET = 0;
    public static final double LEFT_OFFSET = Math.PI/2;
    public static final double BACK_OFFSET = Math.PI;
    public static final double RIGHT_OFFSET = 3* Math.PI / 2;

    public static final double SLOW_SPEED = .1;

    // Stateful
    private Telemetry telemetry;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;


    public static double MAX_TURN = .20;
    public static double MAX_SPEED = .80;
    private boolean isSlow = false;

    /*
     * The angle used to offset the front of the robot
     */
    public double offsetAngle;

    /*
     * Whether or not to log telemetry data
     */
    public boolean silent;

    /*
     * Builds new OmniWheelDriver using default names for motors
     * ("FrontLeft","FrontRight","BackLeft","BackRight")
     * @param {HardwareMap} hardwareMap
     * @param {Telemetry} [telemetry]
     */
    public static OmniWheelDriver build(OpMode opMode) {
        return new OmniWheelDriver(
                opMode.hardwareMap.dcMotor.get("FrontLeft"),
                opMode.hardwareMap.dcMotor.get("FrontRight"),
                opMode.hardwareMap.dcMotor.get("BackLeft"),
                opMode.hardwareMap.dcMotor.get("BackRight"),
                opMode.telemetry
        );
    }

    /*
     * creates new OmniWheelDriver.
     * @param {DcMotor} frontLeft
     * @param {DcMotor} frontRight
     * @param {DcMotor} backLeft
     * @param {DcMotor} backRight
     * @param {Telemetry} telemetry
     */
    public OmniWheelDriver(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft,
                           DcMotor backRight, Telemetry telemetry) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.telemetry = telemetry;
    }

    public void driveold(double x, double y, double rotate, boolean smooth){
        double FL, FR, BL, BR, angle, r;
        //## CALCULATE VALUES ##

        rotate *= MAX_TURN;

        // Takes regular x,y coordinates and converts them into polar (angle radius) cooridnates
        // Then turns angle by 90 degrees (Pi/4) to accommodate omni wheel axis

        // if x is 0, atan comes out undefined instead of PI/2 or 3PI/2

        if (x != 0) {
            angle = Math.atan(y / x);

        }else if(y > 0){//if it's 90 degrees use PI/2
            angle = Math.PI/2;

        }else{
            angle = (3 * Math.PI)/2;
        }

        r = Math.sqrt( (x*x) + (y*y) ) ;//get the radius (hypotenuse)
        angle += (Math.PI/4);//take our angle and shift it 90 deg (PI/4)


        if(smooth || isSlow) r = r*r; //Using a function on variable r will smooth out the slow values but still give full range

        //TODO: r = -(4/3*r-2)/((4/3*r)*(4/3*r)); Cooler more impressive function

        // BUG FIX atan() assumes x is always positive and angle in standard position
        // add PI to go to quadrant 2 or 3
        if(x<0) {
            angle += Math.PI;
        }


        FL = BR =  Math.sin(angle + offsetAngle) * MAX_SPEED * r; //takes new angle and radius and converts them into the motor values
        FR = BL = Math.cos(angle + offsetAngle) * MAX_SPEED * r;

        FL -= rotate; // implements rotation
        FR -= rotate;
        BL += rotate;
        BR += rotate;

        double SPEED = 1;

        if(isSlow)
            SPEED = SLOW_SPEED;


        if(FL<=1 & FR<=1 & BR <=1 & BL<=1) {// Prevent fatal error
            frontLeft.setPower(FL*SPEED); // -rot fl br y
            frontRight.setPower(FR*SPEED); // -
            backLeft.setPower(-BL*SPEED); // +
            backRight.setPower(-BR*SPEED); //+
        }
    }

    /*
     * moves the robot based off of analogue inputs
     * @param {double} x                The x value
     * @param {double} y                The y value
     * @param {double} rotation         The rotation value
     * @param {double} [modifier]      The modifier for the power.
     * @param {boolean} [smooth]        Whether or not to smooth the modifier
     */
    public void drive(double x, double y, double rotation) {
        //Default modifier
        drive(x,y,rotation,Math.sqrt((x*x) + (y*y)), true);
    }
    public void drive(double x, double y, double rotation, boolean smooth) {
        //Default modifier
        drive(x,y,rotation,Math.sqrt((x*x) + (y*y)), smooth);
    }
    public void drive(double x, double y, double rotation, double modifier, boolean smooth) {
        Range.throwIfRangeIsInvalid(x, -1, 1);
        Range.throwIfRangeIsInvalid(y, -1, 1);
        Range.throwIfRangeIsInvalid(rotation, -1, 1);

        //Using a function on variable r will smooth out the slow values but still give full range
        if(smooth)
            modifier = modifier*modifier;

        if(!silent) {
            telemetry.addData("OmniWheelDriver", "x=" + x);
            telemetry.addData("OmniWheelDriver", "y=" + y);
        }

        double angle = 0;
        // if x is 0, atan comes out undefined instead of PI/2 or 3PI/bo
        if (x != 0) {
            angle = Math.atan(y / x);
            if(x<0) {
                angle += Math.PI;
            }
        } else if(y > 0)//if it's 90 degrees use PI/2
            angle = Math.PI/2;
        else {
            angle = (3 * Math.PI) / 2;
        }

<<<<<<< HEAD
//        move(Math.atan2(y, x), rotation, modifier);
        move(angle, rotation, modifier);
    }

    /*
     * moves the robot in the direction specified
     * @param {double} angle
     * @param {double} rotation
     * @param {double} modifier
=======
        //Using the quadratic function on the magnitude
        // will smooth out the slow values but still give full range
        if(smooth)
            magnitude = magnitude*magnitude;
        move(angle, rotation, magnitude);
        //TODO: TRY THIS
        //move(Math.atan2(y, x), rotation, magnitude);
    }

    public boolean getIsSlow() {
        return isSlow;
    }

    public void setIsSlow(boolean isSlow) {
        this.isSlow = isSlow;
    }

    public void switchSlow() {
        isSlow = !isSlow;
    }

    /*
     * Controls the wheel motors based on angle, rotation, and magnitude
>>>>>>> b46745c2dbfdff26d0efa057ca47a14cb09e9ded
     */
    public void move(double angle, double rotation, double modifier) {
        if(!silent) {
            telemetry.addData("OmniWheelDriver", "rotation=" + rotation);
            telemetry.addData("OmniWheelDriver", "angle=" + angle);
            telemetry.addData("OmniWheelDriver", "angle(corrected)=" + (angle + OMNI_WHEEL_ANGLE_CORRECTION));
            telemetry.addData("OmniWheelDriver", "angle(corrected & offset)=" + (angle + OMNI_WHEEL_ANGLE_CORRECTION + offsetAngle));
            telemetry.addData("OmniWheelDriver", "modifier=" + modifier);
            telemetry.addData("OmniWheelDriver", "offset angle=" + offsetAngle);
        }

        angle += OMNI_WHEEL_ANGLE_CORRECTION + offsetAngle;

        frontLeft.setPower(calculateMotorPower(true, true, angle, rotation, modifier));
        frontRight.setPower(calculateMotorPower(true, false, angle, rotation, modifier));
        backLeft.setPower(calculateMotorPower(false, true, angle, rotation, modifier));
        backRight.setPower(calculateMotorPower(false, false, angle, rotation, modifier));
    }
    
    private double calculateMotorPower(boolean isFront, boolean isLeft, double angle, double rotation, double modifier) {
        double power = (isFront == isLeft ? Math.sin(angle) : Math.cos(angle)) * modifier;
        
        if(isFront){
            power -= rotation;
        }
        else {
            power += rotation;
            power *= -1;
        }
        
        return Range.clip(power, -1, 1);
    }

    public void setOffsetAngle(double angle) {
        offsetAngle = angle;
    }

    public void setTelemetry(boolean telemetry) {
        silent = telemetry;
    }
}

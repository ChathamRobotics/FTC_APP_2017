package org.chathamrobotics.ftcutils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Driving with omni wheels
 */
public class OmniWheelDriver implements Driver{
//    CONSTANTS     //
    public static final double OMNI_WHEEL_ANGLE_CORRECTION = Math.PI/4;
    public static final double FRONT_OFFSET = 0;
    public static final double LEFT_OFFSET = Math.PI/2;
    public static final double BACK_OFFSET = Math.PI;
    public static final double RIGHT_OFFSET = 3* Math.PI / 2;

//    STATEFUl      //
    private Telemetry telemetry;
    private DcMotor frontLeft, frontRight, backLeft, backRight;


    // TODO: 12/11/2016 oz add some comments about this stuff. Also u might want to make is slow public for simplicities sake
    public static double MAX_TURN = .3;
    public static double MAX_SPEED = .7;
    public static final double SLOW_SPEED = .35;
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
    public static  OmniWheelDriver build(HardwareMap hardwareMap, Telemetry telemetry) {
        return new OmniWheelDriver(
                hardwareMap.dcMotor.get("FrontLeft"),
                hardwareMap.dcMotor.get("FrontRight"),
                hardwareMap.dcMotor.get("BackLeft"),
                hardwareMap.dcMotor.get("BackRight"),
                telemetry
        );
    }


    /**
     * creates new OmniWheelDriver.
     * @param frontLeft {DcMotor} - front left motor
     * @param frontRight {DcMotor} - front right motor
     * @param backLeft {DcMotor} - back left motor
     * @param backRight {DcMotor} - back right motor
     * @param telemetry {Telemetry} - telemetry
     */
    public OmniWheelDriver(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft,
                           DcMotor backRight, Telemetry telemetry) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.telemetry = telemetry;
    }




    public void stop(){
        driveold(0,0,0);
    }


    /**
     * Team 11248's driving method for omni wheel drive
     * @param x - values of x to drive (double -1 to +1)
     * @param y - values of y to drive (double -1 to +1)
     * @param rotate - values for rotation to drive (double -1 to +1)
     * @param smooth - boolean declaring if driving values are smoothed (low values easier to control)
     */

    public void driveold(double x, double y, double rotate, boolean smooth){
        double FL, FR, BL, BR, angle, r;
        double MAX_TURN, MAX_SPEED;
        //## CALCULATE VALUES ##

        /* This makes the rotation and speed ratios relative to the rotation value
         * So when we don't have a rotation we can drive at full speed instead of a fraction of speed
         * If rotation is being used, a ratio is induced to prevent a value greater than 1
         */

        if (smooth) {
            MAX_TURN = Math.abs(rotate) * this.MAX_TURN;
            MAX_SPEED = 1 - MAX_TURN;

        } else {
            MAX_SPEED = this.MAX_SPEED;
            MAX_TURN = this.MAX_TURN;
        }


        //Using a function on variable rotate will smooth out the slow values but still give full range
        if((smooth || isSlow) && rotate !=0) rotate = rotate * rotate * rotate/Math.abs(rotate);

        rotate *= MAX_TURN;


        angle = Math.atan2(y, x);
//
//        // Takes regular x,y coordinates and converts them into polar (angle radius) cooridnates
//        // Then turns angle by 45 degrees (Pi/4) to accommodate omni wheel axis
//
//        // if x is 0, atan comes out undefined instead of PI/2 or 3PI/2
//
//        if (x != 0) {
//            angle = Math.atan(y / x);
//
//        }else if(y > 0){//if it's 90 degrees use PI/2
//            angle = Math.PI/2;
//
//        }else{
//            angle = (3 * Math.PI)/2;
//        }
//
//        // BUG FIX atan() assumes x is always positive and angle in standard position
//        // add PI to go to quadrant 2 or 3
//        if(x<0) {
//            angle += Math.PI;
//        }


        /* Gets the radius of our left joystick to vary our total speed
        * Checks if r is greater than 1 (cannot assume joystick gives perfect circular values)
        */
        r = Math.sqrt( (x*x) + (y*y) );
        if(r>1) r=1;

        angle += (Math.PI/4);//take our angle and shift it 90 deg (PI/4)


        if(smooth || isSlow) r = r*r; //Using a function on variable r will smooth out the slow values but still give full range

        //TODO: r = -(4/3*r-2)/((4/3*r)*(4/3*r)); Cooler more impressive function




        double SPEED = 1;

        if(isSlow)
            SPEED = SLOW_SPEED;


        /* Takes new angle and radius and converts them into the motor values
         * Multiples by our speed reduction ratio and our slow speed ratio
         */
        FL = BR =  Math.sin(angle + offsetAngle) * MAX_SPEED * r;
        FR = BL = Math.cos(angle + offsetAngle) * MAX_SPEED * r ;

        FL -= rotate; // implements rotation
        FR -= rotate;
        BL += rotate;
        BR += rotate;


        /* Prevent fatal error cause by slightly impefect joystick values
         * Will drive in aproxamite direction if true
         */
        if(Math.abs(FL) >1 || Math.abs(FR) >1 ||
                Math.abs(BR) >1 || Math.abs(BL) >1) {

            FL /=  Math.abs(FL);
            FR /=  Math.abs(FR);
            BR /=  Math.abs(BR);
            BL /=  Math.abs(BL);
        }


            frontLeft.setPower(FL * SPEED); // -rot fl br y
            frontRight.setPower(FR * SPEED); // -
            backLeft.setPower(-BL * SPEED); // +
            backRight.setPower(-BR * SPEED); //+


        if (silent) {
            telemetry.addData("OMNI_DRIVER", "radius: " + r);
            telemetry.addData("OMNI_DRIVER", "x: " + x);
            telemetry.addData("OMNI_DRIVER", "y: " + y);
            telemetry.addData("OMNI_DRIVER", "rotate: " + rotate);
            telemetry.addData("OMNI_DRIVER", "FL: " + frontLeft.getPower());
            telemetry.addData("OMNI_DRIVER", "FR: " + frontRight.getPower());
            telemetry.addData("OMNI_DRIVER", "BR: " + backRight.getPower());
            telemetry.addData("OMNI_DRIVER", "BL: " + backLeft.getPower());
            telemetry.update();
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

    public void driveold(double x, double y,double rotate){
        this.driveold(x, y, rotate, false);
    }

    public void drive(double x, double y, double rotation) {
        //Default modifier
        drive(x,y,rotation,Math.sqrt((x*x) + (y*y)), true);
    }
    public void drive(double x, double y, double rotation, boolean smooth) {
        //Default modifier
        drive(x,y,rotation,Math.sqrt((x*x) + (y*y)), smooth);
    }
    public void drive(double x, double y, double rotation, double modifier) {
        drive(x, y, rotation, modifier, false);
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

        move(angle, rotation, modifier);
    }

    /*
     * returns the value for isSlow
     */
    public boolean getIsSlow() {
        return isSlow;
    }

    /*
     * Sets the value for isSlow
     */
    public void setIsSlow(boolean isSlow) {
        this.isSlow = isSlow;
    }

    /*
     * Toggle isSlow
     */
    public void toggleSlow() {
        isSlow = !isSlow;
    }

    /*
     * moves the robot in the direction specified
     * @param {double} angle
     * @param {double} rotation
     * @param {double} modifier
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

    /*
     * Sets the offset angle
     * @param angle     the angle to offset by
     */
    public void setOffsetAngle(double angle) {
        offsetAngle = angle;
    }

    /*
     * Set the telemetry to silent or not
     * @param telemetry     whether or not to silence telemetry
     */
    public void setTelemetry(boolean telemetry) {
        silent = telemetry;
    }
}

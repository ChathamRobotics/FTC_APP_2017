package org.firstinspires.ftc.team11248.testModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Tony_Air on 3/5/17.
 */

public class ForAditya {

    public static double MAX_SPEED = .8;
    public static double MAX_TURN = .2;


    DcMotor FrontLeft, FrontRight, BackLeft, BackRight;

    public ForAditya(DcMotor FrontLeft, DcMotor FrontRight, DcMotor BackLeft, DcMotor BackRight){

        this.FrontLeft = FrontLeft;
        this.FrontRight = FrontRight;
        this.BackLeft = BackLeft;
        this.BackRight = BackRight;
    }

    public void stop(){
        drive(0,0,0);
    }


    /**
     * drive method to take x,y power to drive holonomic drive chassis
     * takes cartesian power coordinates, converts to polar coordinates,
     * rotates axis 45 degrees, and converts back to cartesian axis for correct motor power
     *
     * @param x - power in the x direction (+ -)
     * @param y - power in the y direction (+ -)
     * @param rotate - power given to rotation (+ -)
     */

    public void drive(double x, double y, double rotate){
        double FL, FR, BL, BR, angle, r;
        double MAX_TURN, MAX_SPEED;


        //## CALCULATE VALUES ##

        /*
         * A ratio is induced to prevent a value greater than 1
         */

        MAX_SPEED = this.MAX_SPEED;
        MAX_TURN = this.MAX_TURN;

        rotate *= MAX_TURN;


        // Takes regular x,y coordinates and converts them into polar (angle radius) cooridnates
        // Then turns angle by 45 degrees (Pi/4) to accommodate omni wheel axis
        angle = Math.atan2(y, x);
        angle += (Math.PI/4);


        /* Gets the radius of our left joystick to vary our total speed
        * Checks if r is greater than 1 (cannot assume joystick gives perfect circular values)
        */
        r = Math.sqrt( (x*x) + (y*y) );
        if(r>1) r=1;



        /* Takes new angle and radius and converts them baxk into the motor values cartesian coordinates
         * Multiples by our speed reduction ratio and our slow speed ratio
         */

        FL = BR = Math.sin(angle) * MAX_SPEED * r;
        FR = BL = Math.cos(angle) * MAX_SPEED * r;

        FL -= rotate; // implements rotation
        FR -= rotate;
        BL += rotate;
        BR += rotate;


        /* Prevent fatal error cause by slightly imperfect joystick values
         * Will drive in approximate direction if true
         */

        FrontLeft.setPower( Range.clip(FL, -1, 1));
        FrontRight.setPower( Range.clip(FR, -1, 1));
        BackLeft.setPower( Range.clip(-BL, -1, 1));
        BackRight.setPower( Range.clip(-BR, -1, 1));
    }
}

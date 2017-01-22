package org.chathamrobotics.ftcutils;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Map;

/**
 * a basic teleop template
 */

public abstract class TeleOpMode extends OpMode {
//    COMPONENTS    //
    protected Robot robot;


//    STATEFUL      //

    /*
     * Whether the current team is red
     */
    public boolean isRedTeam;


//    METHODS       //

    /**
     * returns the team specific robot
     * @return the robot object
     */
    abstract public void buildRobot();
    /*
     * Initializes robot
     */
    public void init() {
        buildRobot();

        if(this.robot == null) {
            Log.wtf("RobotLog", "No Robot Found");
        }

//        this.robot.initHardware();
    }

    /*
     * called on stop
     */
    public void stop() {
        this.robot.stop();
    }
}

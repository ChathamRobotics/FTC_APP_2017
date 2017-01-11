package org.chathamrobotics.ftcutils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Map;

/**
 * a basic teleop template
 */

public abstract class TeleOpMode extends OpMode {
//    COMPONENTS    //
    public Robot robot;


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
    abstract public Robot buildRobot();
    /*
     * Initializes robot
     */
    public void init() {
        this.robot.initHardware();
    }

    /*
     * called on stop
     */
    public void stop() {
        this.robot.stop();
    }
}

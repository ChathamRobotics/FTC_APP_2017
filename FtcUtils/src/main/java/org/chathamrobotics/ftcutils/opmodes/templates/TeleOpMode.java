package org.chathamrobotics.ftcutils.opmodes.templates;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.chathamrobotics.ftcutils.Robot;

import java.util.Map;

/**
 * a basic teleop template
 */

public abstract class TeleOpMode extends OpMode {
//    COMPONENTS    //


//    STATEFUL      //

    /*
     * Whether the current team is red
     */
    public boolean isRedTeam;


//    METHODS       //

    /*
     * get the robot object
     * @return  the robot
     */
    abstract public Robot robot();

    /*
     * Initializes robot
     */
    public void init() {
        robot();
    }

    /*
     * called on stop
     */
    public void stop() {
        this.robot().stop();
    }
}

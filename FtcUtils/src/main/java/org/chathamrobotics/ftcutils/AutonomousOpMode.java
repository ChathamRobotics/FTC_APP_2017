package org.chathamrobotics.ftcutils;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;

import java.util.Map;

/**
 * basic autonomous
 */
public abstract class  AutonomousOpMode extends LinearOpMode {
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
     * Called on start
     */
    abstract public void runRobot() throws StoppedException, InterruptedException;

    /*
     * Initializes robot
     */
    public void initRobot() {

//        this.robot.initHardware();
    }

    /*
     * called on stop
     */
    public void stopRobot() {
        this.robot.stop();
    }

    /*
     * Runs OpMode. Duh!
     */
    @Override
    public void runOpMode() throws InterruptedException {
        buildRobot();


        // Wait for start call
        waitForStart();

        this.robot.debug();

        try {
            runRobot();
        }
        catch (StoppedException error) {
            //Just continue to robot stop
        }
        finally {
            this.robot.debug();
            stopRobot();
        }
    }

    /*
     * periodically checks for stop and updates telemetry
     */
    public void statusCheck() throws StoppedException {
        this.robot.debug();
        checkForStop();
    }

    /*
     * Checks if opmode is still active and if it's not throws a StoppedException
     */
    public void checkForStop() throws StoppedException{
        if (! opModeIsActive()) throw new StoppedException();
    }
}

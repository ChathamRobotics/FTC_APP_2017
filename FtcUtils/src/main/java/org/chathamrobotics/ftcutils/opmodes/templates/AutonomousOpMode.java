package org.chathamrobotics.ftcutils.opmodes.templates;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.chathamrobotics.ftcutils.Robot;
import org.chathamrobotics.ftcutils.StoppedException;

/**
 * basic autonomous
 */
public abstract class  AutonomousOpMode extends LinearOpMode {
//    COMPONENTS    //


//    STATEFUL      //
    /*
     * Whether the current team is red
     */
    protected boolean isRedTeam;


//    METHODS       //
    /*
     * Called on start
     */
    abstract public void runRobot() throws StoppedException, InterruptedException;

    /**
     * get the robot object
     * @return the robot
     */
    abstract public Robot robot();

    /*
     * Initializes robot
     */
    public void initRobot() {
        robot();
    }

    /*
     * called on stop
     */
    protected void stopRobot() {
        this.robot().stop();
    }

    /*
     * Runs OpMode. Duh!
     */
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        // Wait for start call
        waitForStart();

        this.robot().debug();

        try {
            runRobot();
        }
        catch (StoppedException error) {
            //Just continue to robot stop
        }
        finally {
            this.robot().debug();
            stopRobot();
        }
    }

    /*
     * periodically checks for stop and updates telemetry
     */
    protected void statusCheck() throws StoppedException {
        this.robot().debug();
        checkForStop();
    }

    /*
     * Checks if opmode is still active and if it's not throws a StoppedException
     */
    protected void checkForStop() throws StoppedException{
        if (! opModeIsActive()) throw new StoppedException();
    }
}

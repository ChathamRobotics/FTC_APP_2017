package org.firstinspires.ftc.team9853;

import org.chathamrobotics.ftcutils.AutonomousOpMode;
import org.chathamrobotics.ftcutils.StoppedException;


/**
 * backup autonomous
 */
public class AutoModeCharge extends AutonomousOpMode {
    static final long waitTime = 10000;
    static final long driveTime = 3000;

    /*
     * Setup OpMode
     * @param {boolean} isRedTeam   Whether the current team is red
     */
    public AutoModeCharge(boolean isRedTeam) {
        this.isRedTeam = isRedTeam;
    }

    /*
     * called on start
     */
    public void runRobot() throws StoppedException {
        for(long endTime = System.currentTimeMillis() + waitTime; System.currentTimeMillis() < endTime;) {
            statusCheck();
            // Do nothing
        }

        for(long endTime = System.currentTimeMillis() + driveTime; System.currentTimeMillis() < endTime;) {
            statusCheck();
<<<<<<< HEAD
            driver.move(Math.toRadians(isRedTeam ? 135 : 45), 0, .7);
=======
            driver.drive(isRedTeam ? -1 : 1, 1, .7, false);
>>>>>>> b46745c2dbfdff26d0efa057ca47a14cb09e9ded
        }
    }
}

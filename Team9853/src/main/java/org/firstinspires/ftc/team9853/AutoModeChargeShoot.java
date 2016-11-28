package org.firstinspires.ftc.team9853;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.chathamrobotics.ftcutils.AutonomousOpMode;
import org.chathamrobotics.ftcutils.StoppedException;

/**
 * Created by storm on 11/20/2016.
 */

public class AutoModeChargeShoot extends AutonomousOpMode {
    private DcMotor shooter;

    static final long waitTime = 10000;
    static final long shootTime = 1000;
    static final long driveTime = 3000;

    /*
     * Setup OpMode
     * @param {boolean} isRedTeam   Whether the current team is red
     */
    public AutoModeChargeShoot(boolean isRedTeam) {
        this.isRedTeam = isRedTeam;
    }

    @Override
    public void initRobot() {
        super.initRobot();

        shooter = hardwareMap.dcMotor.get("Shooter");
    }

    /*
         * called on start
         */
    public void runRobot() throws StoppedException {
        for(long endTime = System.currentTimeMillis() + shootTime; System.currentTimeMillis() < endTime;) {
            statusCheck();
            shooter.setPower(-1);
        }
        shooter.setPower(0);

        for(long endTime = System.currentTimeMillis() + waitTime - shootTime; System.currentTimeMillis() < endTime;) {
            statusCheck();
            // Do nothing
        }

        for(long endTime = System.currentTimeMillis() + driveTime; System.currentTimeMillis() < endTime;) {
            statusCheck();
            driver.move(90, 0, .7);
        }
    }
}

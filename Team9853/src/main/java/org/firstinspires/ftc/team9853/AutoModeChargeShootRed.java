package org.firstinspires.ftc.team9853;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.chathamrobotics.ftcutils.AutonomousOpMode;
import org.chathamrobotics.ftcutils.StoppedException;

/**
 * Created by storm on 11/20/2016.
 */

@Autonomous(name = "AutoModeChargeShootRed", group = "General")

public class AutoModeChargeShootRed extends AutoModeChargeShoot {
    /*
     * Setup OpMode
     * @param {boolean} isRedTeam   Whether the current team is red
     */
    public AutoModeChargeShootRed() {
        super(true);
    }
}

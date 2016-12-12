package org.firstinspires.ftc.team9853.autonomous.variants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team9853.autonomous.AutoModeBeacons;

/**
 * autonomous beacon presser
 */

@Autonomous(name = "AutoModeBeaconsRed", group = "General")

@Disabled

public class AutoModeBeaconsRed extends AutoModeBeacons {
    AutoModeBeaconsRed() {
        super(false);
    }
}

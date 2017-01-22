package org.firstinspires.ftc.team9853.opmodes;

import org.chathamrobotics.ftcutils.TeleOpMode;
import org.firstinspires.ftc.team9853.Robot9853;

/**
 * team specific template
 */

public abstract class Tele extends TeleOpMode {
//    COMPONENTS    //
    public Robot9853 robot;

//    METHODS       //
    public void buildRobot() {
        this.robot = new Robot9853(hardwareMap, telemetry);
    }
}

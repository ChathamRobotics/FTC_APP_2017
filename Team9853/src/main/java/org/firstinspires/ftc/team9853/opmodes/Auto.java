package org.firstinspires.ftc.team9853.opmodes;

import org.chathamrobotics.ftcutils.opmodes.templates.AutonomousOpMode;
import org.firstinspires.ftc.team9853.Robot9853;

/**
 * team specific template
 */

public abstract class Auto extends AutonomousOpMode {
//    COMPONENTS    //
    private Robot9853 robot;

//    METHODS       //

    public Robot9853 robot() {
        if(robot == null) this.robot = new Robot9853(hardwareMap, telemetry);

        return this.robot;
    }
}

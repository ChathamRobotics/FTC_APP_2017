package org.firstinspires.ftc.team9853.opmodes;

import org.chathamrobotics.ftcutils.opmodes.templates.TeleOpMode;
import org.firstinspires.ftc.team9853.Robot9853;

/**
 * team specific template
 */

public abstract class Tele9853 extends TeleOpMode {
//    COMPONENTS    //
    private Robot9853 robot;

//    METHODS       //

    /**
     * gets the robot object
     * @return the robot
     */
    @Override
    public Robot9853 robot() {
        if(this.robot == null) this.robot = new Robot9853(hardwareMap, telemetry);

        return this.robot;
    }
}

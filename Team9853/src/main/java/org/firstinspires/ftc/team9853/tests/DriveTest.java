package org.firstinspires.ftc.team9853.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.chathamrobotics.ftcutils.Robot;
import org.chathamrobotics.ftcutils.TeleOpMode;
import org.firstinspires.ftc.team9853.Robot9853;
import org.firstinspires.ftc.team9853.opmodes.Tele;

/**
 * Created by storm on 11/15/2016.
 */

@TeleOp(name = "Test: Driving", group = "Test")

@Disabled

public class DriveTest extends Tele {
    DriveTest() {
        super();
    }

    @Override
    public void loop() {
        robot.driveAtPoint(1, 1, 0);

        robot.debug();
    }
}

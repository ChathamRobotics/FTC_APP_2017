package org.firstinspires.ftc.team9853.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.chathamrobotics.ftcutils.StoppedException;
import org.firstinspires.ftc.team9853.opmodes.Auto9853;

/**
 * test gyro
 */

@Autonomous(name = "Test: Gyro", group = "Test")

public class GyroTest extends Auto9853 {
    @Override
    public void runRobot() throws StoppedException, InterruptedException {
        while(robot().driveWithHeading(Math.PI/2, .5, robot().startingHeading)) statusCheck();
        robot().stopDriving();
    }
}

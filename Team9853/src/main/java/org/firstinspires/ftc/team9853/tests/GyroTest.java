package org.firstinspires.ftc.team9853.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.chathamrobotics.ftcutils.Robot;
import org.chathamrobotics.ftcutils.StoppedException;
import org.firstinspires.ftc.team9853.opmodes.Auto9853;

/**
 * test gyro
 */

@Autonomous(name = "Test: Gyro", group = "Test")

public class GyroTest extends Auto9853 {
    public GyroTest() {super(false);}

    @Override
    public void runRobot() throws StoppedException, InterruptedException {
        while(robot().doUntil(1000000)) {
            robot().driveWithHeading(Robot.Side.FRONT.angle, .5, robot().startingHeading);
            statusCheck();
        }
    }
}
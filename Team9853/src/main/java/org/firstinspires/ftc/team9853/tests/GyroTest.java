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
       // for(long endTime = robot().calcEndTime(2000); robot().doUntil(endTime);) {//TODO: Pls Fix
            statusCheck();
            robot().driveWithHeading(Math.PI/2, .5, robot().startingHeading);
        //}
    }
}

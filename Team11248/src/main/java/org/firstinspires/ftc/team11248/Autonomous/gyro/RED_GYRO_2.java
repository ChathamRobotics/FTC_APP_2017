package org.firstinspires.ftc.team11248.Autonomous.gyro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team11248.Robot11248;

/**
 * red autonomous 100 POINTS
 */
@Autonomous(name = "REDGyro2")
public class RED_GYRO_2 extends GENERIC_GYRO {

    @Override
    public void runOpMode() throws InterruptedException {
        isBlue = false;
        super.runOpMode();
    }
}

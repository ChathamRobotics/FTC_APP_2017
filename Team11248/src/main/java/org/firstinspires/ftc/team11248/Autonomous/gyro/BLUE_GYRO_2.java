package org.firstinspires.ftc.team11248.Autonomous.gyro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team11248.Robot11248;

/**
 * blue autonomous 100 POINTS
 */
@Autonomous(name = "BLUEGyro2")
public class BLUE_GYRO_2 extends GENERIC_GYRO {

    @Override
    public void runOpMode() throws InterruptedException {
        isBlue = true;
        super.runOpMode();
    }
}

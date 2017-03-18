package org.firstinspires.ftc.team11248.testModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team11248.Robot11248;

/**
 * Created by Tony_Air on 3/17/17.
 */

@TeleOp(name = "BANGBANG")

public class RealBangBang extends OpMode {

    Robot11248 robot;

    @Override
    public void init() {
        robot = new Robot11248(hardwareMap,telemetry);
        robot.init();
        robot.openCollector();
    }

    @Override
    public void loop() {

        if(gamepad1.a) robot.shooterOnBang();
        else robot.shooterOff();


    }

    @Override
    public void stop(){

    }
}

package org.firstinspires.ftc.team11248.testModes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import static java.util.concurrent.TimeUnit.*;

/**
 * Created by Tony_Air on 2/24/17.
 */

@TeleOp(group = "", name = "EncoderTest")
public class EncoderTest extends OpMode {

    DcMotor ShooterL, ShooterR;
    final double TOLERANCE = .05;
    boolean isRunning = false;
    double rpm;

    @Override
    public void init() {

        ShooterR = hardwareMap.dcMotor.get("ShooterR");
        ShooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ShooterL = hardwareMap.dcMotor.get("ShooterL");
        ShooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       ShooterL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

//        if(gamepad2.right_trigger > TOLERANCE){
//            Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            Shooter.setPower(1);
//            isRunning = true;
//
//        } else if (isRunning) {
//            Shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            Shooter.setTargetPosition(Shooter.getCurrentPosition() + (1440-(Shooter.getCurrentPosition()%1440)));
//            isRunning = false;
//        }


//        Timer shooterL = new Timer();
//        shooterL.scheduleAtFixedRate(new BangBang(ShooterL, 75), 0, 20);
//
        Timer shooterR = new Timer();
        shooterR.scheduleAtFixedRate(new BangBang(ShooterR, 75), 0, 20);


        telemetry.addData("Position L: ", ShooterL.getCurrentPosition());
        telemetry.addData("Position R: ", ShooterR.getCurrentPosition());
        telemetry.addData("RPM: ",  rpm);

    }


    private class BangBang extends TimerTask {

        DcMotor flywheel;
        int lastEncoder, currentEncoder;
        double targetSpeed;
        long loopSpeed = 20;


        BangBang (DcMotor flywheel, double targetSpeed) {

            flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            currentEncoder = lastEncoder = flywheel.getCurrentPosition();
        }

        public void getSpeed(){

            currentEncoder = flywheel.getCurrentPosition();
            rpm = (double)(currentEncoder-lastEncoder)/ (1440 * loopSpeed * .001);
            lastEncoder = currentEncoder;

            //power = rpm/152.0;
        }

        public void run(){

            getSpeed();

            double absRPM = Math.abs(rpm);

            if(absRPM >= targetSpeed){
                flywheel.setPower(0);

            }else if(absRPM < targetSpeed){
                flywheel.setPower(rpm / absRPM);
            }


        }

    }

}

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
    int targetSpeed = 0;
    int loops = 0;

    ScheduledThreadPoolExecutor shooterR = new ScheduledThreadPoolExecutor(5);



    @Override
    public void init() {

        ShooterR = hardwareMap.dcMotor.get("ShooterR");
        ShooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterR.setDirection(DcMotorSimple.Direction.REVERSE);

        ShooterL = hardwareMap.dcMotor.get("ShooterL");
        ShooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterR.scheduleAtFixedRate(new BangBang(ShooterR), 0, 1, SECONDS);

    }

    @Override
    public void loop() {


        targetSpeed = gamepad1.a?75:0;
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



        telemetry.addData("Position L: ", ShooterL.getCurrentPosition());
        telemetry.addData("Position R: ", ShooterR.getCurrentPosition());
        telemetry.addData("RPM: ",  rpm);
        telemetry.addData("loops: ", loops);

    }


    public class BangBang implements Runnable {

        DcMotor flywheel;
        int lastEncoder, currentEncoder;
        //double targetSpeed;
        long loopSpeed = 20;


        public BangBang (DcMotor flywheel) {

            flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            currentEncoder = lastEncoder = flywheel.getCurrentPosition();
          //  this.targetSpeed = targetSpeed;
        }

        public void getSpeed(){

            currentEncoder = flywheel.getCurrentPosition();
            rpm = (double)(currentEncoder-lastEncoder)/ (1440 * loopSpeed * .001);
            lastEncoder = currentEncoder;

            //power = rpm/152.0;
        }

        public void run(){

            loops++;

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

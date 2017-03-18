package org.firstinspires.ftc.team11248.testModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.team11248.Robot11248;

import static org.firstinspires.ftc.team11248.testModes.Thread_Test.killed;

/**
 * Created by Tony_Air on 3/17/17.
 */

@TeleOp(name = "Threads")
public class Thread_Test extends OpMode {


    public static int loop = 0;
    public static int loop2 = 0;
    public static boolean killed = false;
    public static boolean bangBangOn = false;

    public static final double SPEED = 125;

    static double rpm, rpm1;
    Thread example = new Example(20, true);
    Thread example2 = new Example(20, false);

    public static DcMotor shooter, shooter1;


    static boolean run = false;

    @Override
    public void init() {

        shooter = hardwareMap.dcMotor.get("ShooterL");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Thread bangbang = new BangBangs(shooter);
        new Thread(bangbang).start();

        shooter1 = hardwareMap.dcMotor.get("ShooterR");
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Thread bangbanga = new BangBanga(shooter);
        new Thread(bangbanga).start();



//        new Thread(example2).start();
    }


    @Override
    public void loop() {



        bangBangOn = gamepad1.a;

        telemetry.addData("STATE", rpm);
        telemetry.addData("Loops", loop);
        telemetry.addData("Loops2", loop2);


    }

    @Override
    public void stop(){
        killed = true;
    }



}


class Example extends Thread{

    private int loopTime;

    private boolean isOne;


    Example (int millis, boolean isOne){
        loopTime =  millis;
        this.isOne = isOne;
    }


    @Override
    public void run() {
        while (!isInterrupted() && !Thread_Test.killed) {
            if(Thread_Test.run) {

                if (isOne) Thread_Test.loop++;
                else Thread_Test.loop2++;

                try {
                    Thread.sleep(loopTime);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

            }
        }
    }
}

class BangBangs extends Thread{

    int lastEncoder, currentEncoder;
    long loopSpeed = 20;

    BangBangs (DcMotor flywheel){

        currentEncoder = lastEncoder = Thread_Test.shooter.getCurrentPosition();

    }

    private void getSpeed(){

        lastEncoder = currentEncoder;
        currentEncoder = Thread_Test.shooter.getCurrentPosition();
        Thread_Test.rpm = ((currentEncoder-lastEncoder)/1440.0)/(loopSpeed * .001*.0166666667);
    }

    @Override
    public void run() {

        while (!isInterrupted()) {

            getSpeed();

            Thread_Test.loop2++;


            if (Thread_Test.bangBangOn) {

                Thread_Test.loop++;
                if (Thread_Test.rpm >= Thread_Test.SPEED) {
                    Thread_Test.shooter.setPower(0);

                } else if (Thread_Test.rpm < Thread_Test.SPEED) {
                    Thread_Test.shooter.setPower(1);    //Thread_Test.shooter.setPower(Thread_Test.rpm / Math.abs(Thread_Test.rpm));
                }

            }else{
                Thread_Test.shooter.setPower(0);
            }

            try {
                Thread.sleep(loopSpeed);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}

class BangBanga extends Thread{

    int lastEncoder, currentEncoder;
    long loopSpeed = 20;

    BangBanga (DcMotor flywheel){

        currentEncoder = lastEncoder = Thread_Test.shooter1.getCurrentPosition();

    }

    private void getSpeed(){

        lastEncoder = currentEncoder;
        currentEncoder = Thread_Test.shooter1.getCurrentPosition();
        Thread_Test.rpm1 = ((currentEncoder-lastEncoder)/1440.0)/(loopSpeed * .001*.0166666667);
    }

    @Override
    public void run() {

        while (!isInterrupted()) {

            getSpeed();

           // Thread_Test.loop2++;


            if (Thread_Test.bangBangOn) {

             //   Thread_Test.loop++;
                if (Thread_Test.rpm1 >= Thread_Test.SPEED) {
                    Thread_Test.shooter1.setPower(0);

                } else if (Thread_Test.rpm1 < Thread_Test.SPEED) {
                    Thread_Test.shooter1.setPower(1);    //Thread_Test.shooter.setPower(Thread_Test.rpm / Math.abs(Thread_Test.rpm));
                }

            }else{
                Thread_Test.shooter1.setPower(0);
            }

            try {
                Thread.sleep(loopSpeed);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}

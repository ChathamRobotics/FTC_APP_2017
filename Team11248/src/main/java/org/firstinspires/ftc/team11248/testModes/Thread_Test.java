package org.firstinspires.ftc.team11248.testModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Tony_Air on 3/17/17.
 */

@TeleOp(name = "Threads")
public class Thread_Test extends OpMode {


    public static int loop = 0;
    public static int loop2 = 0;

    Thread example = new Example(20, true);
    Thread example2 = new Example(20, false);

    static boolean run = false;

    @Override
    public void init() {

        new Thread(example).start();
        new Thread(example2).start();
    }


    @Override
    public void loop() {

        run = gamepad1.a;

        //telemetry.addData("STATE", example.getState());
        telemetry.addData("Loops", loop);
        telemetry.addData("Loops2", loop2);
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
        while (!isInterrupted()) {
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

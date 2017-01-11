package org.chathamrobotics.ftcutils;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

/**
 * A Object to abstract away the hardware aspects of the robot. We're building Karel! Robot.turnLeft!
 */

public abstract class Robot {
//    CONSTANTS         //
    protected static final String TAG = "RobotLog";

//    TOOLS             //

    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected Driver driver;

//    CONSTRUCTORS      //

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

//    ABSTRACT METHODS  //

    /**
     * initializes the robots hardware
     */
    public abstract void initHardware();

    /**
     * starts robot. (ex: puts servos in start positions)
     */
    public abstract void start();

//    METHODS           //

    /**
     * This method is used to stop the robot. This should set all motor powers to zero, and do anything else required to stop the robot.
     */
    public void stop() {
        // Stops all the motors. All robots should have this in common
        for (Map.Entry<String, DcMotor> entry : this.hardwareMap.dcMotor.entrySet()) {
            entry.getValue().setPower(0);
        }
    }

    /**
     * This method is used to update the telemetry and robot log
     */
    public void debug() {
        debug(true); // This is here just to make debug easier to call instead of having to do debug(true). If you don't want the telemetry to update when debug is called then do debug(false)
    }
    public void debug(boolean update) {
        String currentLine; //This is so that a new variable isn't created for each loop

        // Debug motor values
        for (Map.Entry<String, DcMotor> entry : this.hardwareMap.dcMotor.entrySet()) {
            // if the motor is moving
//            if(entry.getValue().isBusy()) {}

            currentLine = Double.toString(entry.getValue().getController().getMotorPower(entry.getValue().getPortNumber()));

            // Write to telemetry
            this.telemetry.addData("Motor " + entry.getKey() + " Power", currentLine);

            // Write to android log
            Log.d(this.TAG, "Motor" + entry.getKey() + "Power = " + currentLine);
        }

        // Debug servo values
        for (Map.Entry<String, Servo> entry: this.hardwareMap.servo.entrySet()) {
            currentLine = Double.toString(entry.getValue().getController().getServoPosition(entry.getValue().getPortNumber()));

            // Write to telemetry
            this.telemetry.addData("Servo" + entry.getKey() + " Position", currentLine);

            // Write to android log
            Log.d(this.TAG, "Servo" + entry.getKey() + " Position = " + currentLine);
        }

        // update telemetry values if needed
        if(update) {
            this.telemetry.update();
        }
    }
}

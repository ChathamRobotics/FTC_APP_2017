package org.firstinspires.ftc.team11248.Jason;

/**
 * Created by tonytesoriero on 9/1/17.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    private Telemetry telemetry;
    private DcMotor motor;
    private GyroSensor gyro;
    private Servo dontdie;
    private OpticalDistanceSensor ODC;
    private TouchSensor tlc;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry){

        this.telemetry = telemetry;
        this.motor = hardwareMap.dcMotor.get("kachow");
        this.gyro = hardwareMap.gyroSensor.get("King");
        this.dontdie = hardwareMap.servo.get("servo2");
        this.ODC = hardwareMap.opticalDistanceSensor.get("chick");
        this.tlc = hardwareMap.touchSensor.get("mater");
    }


    //only use values between -1 and 1
    public void setMotorPower(double power){

        if(power == 1){
            motor.setPower(power);

        } else {
            motor.setPower(power);

        }
    }

    //only use values between 0 and 1
    public void setServoPosition(double value){
       dontdie.setPosition(value);
    }

    public boolean crocchow (){
       return tlc.isPressed();
    }


    //calibrate gyro and wait till ift finishes
    public void calibrateGyro(){
        gyro.calibrate();
        while(gyro.isCalibrating()) { }
    }

    public int getGyroAngle(){
        return gyro.getHeading();
    }

    public double getlight (){
        return ODC.getLightDetected();
    }
}

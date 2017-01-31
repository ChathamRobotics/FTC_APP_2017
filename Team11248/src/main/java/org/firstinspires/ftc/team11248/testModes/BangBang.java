package org.firstinspires.ftc.team11248.testModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Tony_Air on 1/25/17.
 */

@TeleOp(name = "BangBangTest")
public class BangBang extends OpMode {



    private double rpm, doubleSpeed, elapsedTime;
    private long lastTime;
    private long lastEncoder, currentEncoder;

    private final double tolerance = .015;
    private final int ticksPerRevolution = 1440;



    private static final double TOLERANCE = 0.00000005;
    private static final double TARGET_VELOCITY = .0000011;

    private DcMotor flywheelLeft,flywheelRight;
    PIDCalculator velocityPID = new PIDCalculator();
    VelocityCalculator flywheelVelocity = new VelocityCalculator();
    BangBangCalculator velocityBangBang = new BangBangCalculator();


    @Override
    public void init() {

        flywheelLeft = hardwareMap.dcMotor.get("ShooterL");
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelRight = hardwareMap.dcMotor.get("ShooterR");
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    @Override
    public void loop() {

        BangBangLoop();

    }


    public void BangBangOld(DcMotor motor, double targetPower, boolean isBangBang){


        long now = System.nanoTime();
        double power = targetPower;

        elapsedTime = (double)(now - lastTime)/60000000000.0;
        currentEncoder = motor.getCurrentPosition();

        rpm = (double)(currentEncoder-lastEncoder)/(double)ticksPerRevolution/elapsedTime;
        doubleSpeed = rpm/152.0;


        if(isBangBang) {
            if (doubleSpeed >= (targetPower + tolerance)) {
                power = targetPower - .05;

            } else if (doubleSpeed <= (targetPower - tolerance)) {
                power = targetPower + .05;
            }

            motor.setPower(Range.clip(power, -1, 1));
        }


        lastEncoder = currentEncoder;
        lastTime = now;
    }


        /*
        PID Example
         */
        private void PIDLoop(){
            flywheelVelocity.setParameters(System.nanoTime(), flywheelRight.getCurrentPosition());
            double currentVelocity = flywheelVelocity.getVelocity();
            double currentError = TARGET_VELOCITY - currentVelocity;

            velocityPID.setParameters(0.37, 0.1, 0.0, currentError, 0.85);
            double motorOut = velocityPID.getPID();

            motorOut = Range.clip(motorOut, 0, 1);
            flywheelLeft.setPower(motorOut);
            flywheelRight.setPower(motorOut);
        }

        /*
        Bang Bang Example
         */
        private void BangBangLoop(){
            flywheelVelocity.setParameters(System.nanoTime(), flywheelRight.getCurrentPosition());
            double currentVelocity = flywheelVelocity.getVelocity();

            velocityBangBang.setParameters(currentVelocity, TARGET_VELOCITY, 0.84, 0.9, TOLERANCE);
            double motorOut = velocityBangBang.getBangBang();

            motorOut = Range.clip(motorOut, 0, 1);
            flywheelLeft.setPower(motorOut);
            flywheelRight.setPower(motorOut);
        }


    public class PIDCalculator
    {
        private double kP, kI, kD, error, constant;
        private double lastError;
        private double integral, derivative;

        public void setParameters(double kP, double kI, double kD, double error, double constant)
        {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.error = error;
            this.constant = constant;
        }

        public double getPID()
        {
            derivative = error - lastError;
            integral += error;
            lastError = error;
            return (kP * error) + (kI * integral) + (kD * derivative) + constant;
        }
    }

    public class BangBangCalculator
    {
        private double target;
        private double velocity;
        private double lowerPower, higherPower;
        private double tolerance;

        public void setParameters(double target, double velocity, double lowerPower, double higherPower, double tolerance)
        {
            this.target = target;
            this.velocity = velocity;
            this.lowerPower = lowerPower;
            this.higherPower = higherPower;
            this.tolerance = tolerance;
        }

        public double getBangBang()
        {
            if(velocity >= (target + tolerance))
            {
                return lowerPower;
            }

            else
            {
                return higherPower;
            }
        }
    }

    public class VelocityCalculator
    {
        private long time, encoder;
        private long lastEncoder, lastTime;

        public void setParameters(long time, long encoder)
        {
            this.time = time;
            this.encoder = encoder;
        }

        public double getVelocity()
        {
            double velocity = (double) (encoder - lastEncoder) / (time - lastTime);

            lastEncoder = encoder;
            lastTime = time;

            return velocity;
        }
    }
}
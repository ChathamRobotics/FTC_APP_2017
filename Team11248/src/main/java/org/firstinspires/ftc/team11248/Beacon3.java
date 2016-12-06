package org.firstinspires.ftc.team11248;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cController.I2cPortReadyCallback;
import com.qualcomm.robotcore.hardware.Servo;

import org.chathamrobotics.ftcutils.MRColorSensorV2;

import java.util.concurrent.locks.Lock;

@TeleOp(name = "Beacon3")
public class Beacon3 extends OpMode implements I2cPortReadyCallback {

    // IMPORTANT!
    // What did you name your color sensor in the Robot Controller config?
    String color_sensor_name = "color3";

    // Color Sensor hardware
    ModernRoboticsI2cColorSensor color_sensor;
    I2cController controller;

    // What did you name your color sensor in the Robot Controller config?
    String color_sensor_name2 = "color2";

    // Color Sensor hardware
    ModernRoboticsI2cColorSensor color_sensor2;
    I2cController controller2;

    // Variable to track the read/write mode of the sensor
    I2CMode controller_mode = I2CMode.READ;

    // Variables to prevent repeat calibration
    boolean isWorking = false;
    double timeStartedWorking = 0.0;

    // I2C address, registers, and commands
    public byte COLOR_SENSOR_ADDR = 0x3C;
    public byte WRITE_CACHE_OFFSET = 4;

    public Robot11248 robot;

    /* In init(), we get a handle on the color sensor itself and its controller.
     * We need access to the cycle of events on the sensor (when the port is
     * ready for read/write activity). The portIsReady() method is registered
     * as a callback.
     */
    public void init() {

        DcMotor[] motors = new DcMotor[8];
        Servo[] servos = new Servo[1];
        MRColorSensorV2[] colors = new MRColorSensorV2[3];
        for(int i = 0; i < motors.length; i++)
            motors[i] = hardwareMap.dcMotor.get(Robot11248.MOTOR_LIST[i]);
        for(int i = 0; i < servos.length; i++)
            servos[i] = hardwareMap.servo.get(Robot11248.SERVO_LIST[i]);
        robot = new Robot11248(motors,servos,telemetry);
        robot.init(); //Sets servos to right position.

        // Remember to change the name of the color sensor in get().
        color_sensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get(color_sensor_name);

        // Get a handle on the color sensor's controller.
        controller = color_sensor.getI2cController();

        // Register the callback for portIsReady().
        controller.registerForI2cPortReadyCallback(this, color_sensor.getPort());

        // Remember to change the name of the color sensor in get().
        color_sensor2 = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get(color_sensor_name);

        // Get a handle on the color sensor's controller.
        controller2 = color_sensor2.getI2cController();

        // Register the callback for portIsReady().
        controller2.registerForI2cPortReadyCallback(this, color_sensor2.getPort());

//        // Unimportant.
//        telemetry.setSorted(false);
    }

    public static boolean isNeighborColor(int r, int g, int b,
                                          int r2, int g2, int b2, int tolerance) {
        return Math.abs(r - r2) <= tolerance
                && Math.abs(g - g2) <= tolerance
                && Math.abs(b2 - b2) <= tolerance;
    }

    boolean foundWhite1;
    boolean stop1;

    // Respond to gamepad input.
    public void loop() {

        if(!foundWhite1)
            robot.driveold(.5,.5,0,false);
        if(!stop1)



        // If enough time has elapsed since the last time we started a
        // calibration, allow another one to be started.
        if (isWorking) {

            // The if-statements are written in this way so that no
            // telemetry is written while an operation is taking place.

            // Reset the flag.
            if (getRuntime() - timeStartedWorking > 2.0)
                isWorking = false;
        }

        // Give instructions to the user.
        else {
            String reading = "R(" + color_sensor.red() +
                    ") G(" + color_sensor.green() +
                    ") B(" + color_sensor.blue() +
                    ") A(" + color_sensor.alpha() + ")";

            telemetry.addData("Reading", reading);
        }
    }


    /* In sendCommand(), we do the write process. Before reading or writing
     * anything, we need to lock the relevant cache. We enable write mode,
     * write our command code to the write cache, and then signal the
     * controller to re-enable read mode when the port is next ready.
     */
    public synchronized void sendCommand(byte command) {

        // Get a handle on the write cache and lock.
        int port = color_sensor.getPort();
        Lock wLock = controller.getI2cWriteCacheLock(port);
        byte[] wCache = controller.getI2cWriteCache(port);

        // Do the locking in a try/catch, in case a lock can't be made.
        try {

            // Lock the cache before anything.
            wLock.lock();

            // Enable write mode on the controller.
            controller.enableI2cWriteMode(port, I2cAddr.create8bit(COLOR_SENSOR_ADDR), 0x03, 1);

            // Write the supplied command to the relevant register.
            wCache[WRITE_CACHE_OFFSET] = command;

        } finally {

            // Ensure the cache is unlocked.
            wLock.unlock();

            // Signal portIsReady() to enable read mode when we're done.
            controller_mode = I2CMode.WRITE;
        }
    }


    /* When the I2C port is ready for read/write action, we may need to take
     * different actions depending on what we have queued. We use the
     * controller_mode variable to track the current state.
     */
    public synchronized void portIsReady(int port) {

        // I'm not sure why this needs to take place.
        controller.setI2cPortActionFlag(port);
        controller.readI2cCacheFromController(port);

        switch (controller_mode) {

            // Flush the write cache and set up a reset on next cycle.
            case WRITE:
                controller.writeI2cCacheToController(port);
                controller_mode = I2CMode.RESET;
                break;

            // During reset, we move back to read mode.
            case RESET:
                controller.enableI2cReadMode(port, I2cAddr.create8bit(COLOR_SENSOR_ADDR), 0x03, 6);
                controller.writeI2cCacheToController(port);
                controller_mode = I2CMode.READ;
                break;

            // Let ModernRoboticsI2cColorSensor handle reading.
            case READ:
            default:
                break;
        }

        // Allow the ModernRoboticsI2cColorSensor class to handle the rest of
        // the portIsReady read/write cycle.
        color_sensor.portIsReady(port);
    }


    // Enum for the controller_mode variable.
    private enum I2CMode {
        READ, WRITE, RESET
    }
}
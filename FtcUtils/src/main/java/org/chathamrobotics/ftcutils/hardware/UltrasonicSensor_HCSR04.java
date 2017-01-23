package org.chathamrobotics.ftcutils.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * Created by Tony_Air on 12/20/16.
 */

public class UltrasonicSensor_HCSR04 {

    private DigitalChannel echo, trigger;


    /**
     * Builds new Ultrasonic Sensor based on a the HC-SR04 sensor
     * @param echo - the DigitalChannel of the echo pin
     * @param trigger - the DigitalChannel of the trigger pin
     */

    public UltrasonicSensor_HCSR04(DigitalChannel echo, DigitalChannel trigger){

        this.echo = echo;
        this.trigger = trigger;

        this.echo.setMode(DigitalChannelController.Mode.INPUT);
        this.trigger.setMode(DigitalChannelController.Mode.OUTPUT);

    }



    /**
     * Measures the duration of a digital pulse
     * @param digitalInput - the digital pin capturing the pulse
     * @param state  the state of the pulse peak (if peak of pulse is high, state = true)
     * @param timeoutMicro - the timeout threshold in microseconds (prevents "stuck in loop()" error)
     * @return duration of the pulse in nanoseconds
     */
    public long pulseIn(DigitalChannel digitalInput, boolean state, long timeoutMicro){

        long timeoutNano = timeoutMicro * 1000;
        long startTime = System.nanoTime();
        long startPulse;

        while(digitalInput.getState() == state)
            if(System.nanoTime() - startTime > timeoutNano)
                return 0;

        while(digitalInput.getState() != state)
            if(System.nanoTime() - startTime > timeoutNano)
                return 0;

        startPulse = System.nanoTime();
        while(digitalInput.getState() == state)
            if(System.nanoTime() - startTime > timeoutNano)
                return 0;

        return System.nanoTime() - startPulse;
    }


    /*
     * Takes the duration of the pulse and converts it to inches
     * The ultrasonic's pulse travels at 1130 feet per second
     * Convert this to nanoseconds (what is returned from getRawValue();
     * Divide by 2 to get the distance to the object
     * (because the pulse is time left, reflected, and returned)
     *
     * 1 seconds / 1130 fps / 12 inches * 1,000,000,000 nanoseconds = 73,746 nanoseconds per inch
     */

    public long distanceIN(){
        return getRawValue() / 74746/2;
    }


    /*
     * Takes the duration of the pulse and converts it to centimeters
     * 73,746 nanoseconds per inch / 25.4 mm per in = 29,033 nanoseconds per centimeter
     */

    public long distanceCM(){
        return getRawValue() / 29033/2;
    }


    /*
     * Takes the duration of the pulse and converts it to millimeters
     * 29,033 nanoseconds per centimeter / 10 mm per cm = 2,903 nanoseconds per millimeter
     */

    public long distanceMM(){
        return getRawValue() / 2903/2;
    }


    /*
     * Triggers the sensor and returns the duration from sending and receiving the ultrasonic pulse
     */

    public long getRawValue(){
        triggerSensor();
        return pulseIn(echo, true, 20000);
    }


    /*
     * Sends out a 5 microsecond long pulse on the "trigger" pin to trigger the sensor
     */

    public void triggerSensor(){
        trigger.setState(false);
        sleep(0, 2000);
        trigger.setState(true);
        sleep(0, 6000);
        trigger.setState(false);

    }

    public void setTriggerState(boolean state){
        trigger.setState(state);
    }

    public boolean getEchoState(){
        return echo.getState();
    }

    private void sleep(int millis, int nanos){
        try{
            Thread.sleep(millis,nanos);
        } catch (InterruptedException e){}
    }
}

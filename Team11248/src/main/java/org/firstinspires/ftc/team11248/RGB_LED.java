package org.firstinspires.ftc.team11248;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.PWMOutputImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;

import java.util.Timer;
import java.util.TimerTask;

/**
 * Created by Tony_Air on 1/10/17.
 */

public class RGB_LED {

    public float hue, saturation, value;

    private Servo red, green, blue;

    private Timer fadeLedsTimer;

    private final int fadeInterval = 100;



    public RGB_LED(ServoController servoController){

        this.red = new ServoImpl(servoController, 0);
        this.green = new ServoImpl(servoController, 1);
        this.blue = new ServoImpl(servoController, 2);

//        this.red = new PWMOutputImpl(deviceInterfaceModule, 0);
//        this.green = new PWMOutputImpl(deviceInterfaceModule, 1);
//        this.blue = new AnalogOutput(deviceInterfaceModule, 0);
//
//
//        this.red.setPulseWidthPeriod(255);
//        this.green.setPulseWidthPeriod(255);
//
//        this.blue.setAnalogOutputMode( (byte)2 );
//        this.blue.setAnalogOutputVoltage(5);
    }



    /*
     * SET LED COLOR METHODS
     */



    public void setLedColor(int red, int green, int blue) {

        if (0 > red || red > 255 || 0 > green || green > 255 || 0 > blue || blue > 255) {
            throw (new IllegalArgumentException());
        }

        //TODO: Make sure hsv values are updated aka not {0,0,0}
        float[] hsv = {0,0,0};
        Color.RGBToHSV(red, green, blue, hsv);

        this.red.setPosition(red/255);
        this.green.setPosition(green/255);
        this.blue.setPosition(blue/255);
    }

    public void setLedColor(int color){

        int red = Color.red(color);
        int green = Color.green(color);
        int blue = Color.blue(color);

        setLedColor(red,green, blue);

    }

    public void setLedColor(float[] hsv){

        setLedColor(Color.HSVToColor(hsv));
    }

    public void setLedColor(String color){

        int colorNum = Color.parseColor(color);
        setLedColor(colorNum);
    }



    /*
     * FADE LED METHODS
     */

    public void fadeLed(float[] hsv, int time){

        fadeLedsTimer.cancel();

        fadeLedsTimer = new Timer();
        fadeLedsTimer.schedule(new Fade(hsv, time), 0, fadeInterval);

    }

    public void fadeLed(int red, int green, int blue, int time){

        //TODO: Make sure hsv values are updated aka not {0,0,0}
        float[] hsv = {0,0,0};
        Color.RGBToHSV(red, green, blue, hsv);

        fadeLed(hsv, time);
    }

    public void fadeLed(int color, int time){

        //TODO: Make sure hsv values are updated aka not {0,0,0}
        float[] hsv = {0,0,0};
        Color.colorToHSV(color, hsv);

        fadeLed(hsv, time);
    }

    public void fadeLed(String color, int time){
        int colorNum = Color.parseColor(color);
        fadeLed(colorNum, time);
    }


    /*
     * RAINBOW LED METHODS
     */


    public void rainbowLed(){}


    /*
     * TIMERTASKs (LED ANIMATIONS)
     */

    private class Fade extends TimerTask{

        private float changeIntervalHue, changeIntervalSat, changeIntervalVal;
        private int iteration, totalIteration;
        private float[] hsv = {0,0,0};


        /**
         * Creates an LED fading to animation (timed)
         * @param hsv {float[]} - the target HSV values
         * @param time {int} - number of seconds for animation
         */
        private Fade(float[] hsv, int time) {

            this.hsv[0] = hue;
            this.hsv[1] = saturation;
            this.hsv[2] = value;

            //finds magnitude of value change
            float netHue = hsv[0] - hue;
            float netSat = hsv[1] - saturation;
            float netVal = hsv[2] - value;

            //finds amount of loops needed
            totalIteration = Math.round(time / fadeInterval);
            iteration = 1;

            //sets the amount each value has to change each iteration
            this.changeIntervalHue = netHue / totalIteration;
            this.changeIntervalSat = netSat / totalIteration;
            this.changeIntervalVal = netVal / totalIteration;
        }

        public void run() {

            hsv[0] += changeIntervalHue;
            hsv[1] += changeIntervalSat;
            hsv[2] += changeIntervalVal;

            setLedColor(hsv);
            iteration++;

            if(iteration > totalIteration) System.exit(0);

        }
    }

    public class Rainbow extends TimerTask{

        float interval;
        float[] hsv = {0,0,0};
        int changingVal, direction;

        boolean goingUp = true;


        /**
         * Creates a Rainbow like LED animation (continuous)
         * @param changingVal {int} - (0, 1 or 2) it is the hsv value that will be change
         * @param direction {int} - (0, 1 or 2) direction of animated value 0 = up, 1 = down, 2 = up and down
         * @param interval {float} - amount each value is changed each iteration (speed)
         */
        public Rainbow(int changingVal, int direction, float interval){

            if(0 > changingVal || changingVal > 3) throw(new IllegalArgumentException());
            if(0 > direction || direction > 3) throw(new IllegalArgumentException());

            this.interval = interval;
            this.changingVal = changingVal;

            this.hsv[0] = hue;
            this.hsv[1] = saturation;
            this.hsv[2] = value;
        }

        public void run(){

            switch (direction){
                case 0: //If up

                    hsv[changingVal] += interval;
                    if(hsv[changingVal] > 360) hsv[changingVal] = 0;
                    break;

                case 1: //If down

                    hsv[changingVal] -= interval;
                    if(hsv[changingVal] < 0) hsv[changingVal] = 360;
                    break;

                case 2://If up and down

                    if(goingUp){ //flipflop direction
                        hsv[changingVal] += interval;
                    }else{
                        hsv[changingVal] -= interval;
                    }

                    if(hsv[changingVal] < 0){ //bounce lowbounds
                        hsv[changingVal] = 0;
                        goingUp = true;

                    }else if(hsv[changingVal] > 360){//bounce highbounds
                        hsv[changingVal] = 360;
                        goingUp = true;
                    }
                    break;

            }


            setLedColor(hsv);
        }
    }

}

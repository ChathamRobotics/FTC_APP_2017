package org.chathamrobotics.ftcutils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * GameController utility class
 */

public class GameController {
    private Gamepad gp;

    /**
     * left analog stick horizontal axis
     */
    public AnalogueButton left_stick_x;

    /**
     * left analog stick vertical axis
     */
    public AnalogueButton left_stick_y;

    /**
     * right analog stick horizontal axis
     */
    public AnalogueButton right_stick_x;

    /**
     * right analog stick vertical axis
     */
    public AnalogueButton right_stick_y;

    /**
     * dpad up
     */
    public BinaryButton dpad_up;

    /**
     * dpad down
     */
    public BinaryButton dpad_down;

    /**
     * dpad left
     */
    public BinaryButton dpad_left;

    /**
     * dpad right
     */
    public BinaryButton dpad_right;

    /**
     * button a
     */
    public BinaryButton a;

    /**
     * button b
     */
    public BinaryButton b;

    /**
     * button x
     */
    public BinaryButton x;

    /**
     * button y
     */
    public BinaryButton y;

    /**
     * button guide - often the large button in the middle of the controller. The OS may
     * capture this button before it is sent to the app; in which case you'll never
     * receive it.
     */
    public BinaryButton guide;

    /**
     * button start
     */
    public BinaryButton start;

    /**
     * button back
     */
    public BinaryButton back;

    /**
     * button left bumper
     */
    public BinaryButton left_bumper;

    /**
     * button right bumper
     */
    public BinaryButton right_bumper;

    /**
     * left stick button
     */
    public BinaryButton left_stick_button;

    /**
     * right stick button
     */
    public BinaryButton right_stick_button;

    /**
     * left trigger
     */
    public AnalogueButton left_trigger;

    /**
     * right trigger
     */
    public AnalogueButton right_trigger;
    

    /*
     * Create a new GameController from a game pad
     */
    public GameController(Gamepad gp) {
        this.gp = gp;

        this.a = new BinaryButton(gp.a);
        this.b = new BinaryButton(gp.b);
        this.x = new BinaryButton(gp.x);
        this.y = new BinaryButton(gp.y);

        this.dpad_up = new BinaryButton(gp.dpad_up);
        this.dpad_down = new BinaryButton(gp.dpad_down);
        this.dpad_left = new BinaryButton(gp.dpad_left);
        this.dpad_right = new BinaryButton(gp.dpad_right);

        this.left_stick_x = new AnalogueButton(gp.left_stick_x);
        this.left_stick_y = new AnalogueButton(gp.left_stick_y);
        this.right_stick_x = new AnalogueButton(gp.right_stick_x);
        this.right_stick_y = new AnalogueButton(gp.right_stick_y);

        this.guide = new BinaryButton(gp.guide);
        this.start = new BinaryButton(gp.start);
        this.back = new BinaryButton(gp.back);

        this.left_bumper = new BinaryButton(gp.left_bumper);
        this.right_bumper = new BinaryButton(gp.right_bumper);

        this.left_stick_button = new BinaryButton(gp.left_stick_button);
        this.right_stick_button = new BinaryButton(gp.right_stick_button);

        this.left_trigger = new AnalogueButton(gp.left_trigger);
        this.left_trigger.minValue = 0;

        this.right_trigger = new AnalogueButton(gp.right_trigger);
        this.right_trigger.minValue = 0;
    }

    /*
     * Button
     */
    public abstract class Button<RETURN_TYPE> {
        private RETURN_TYPE btn;
        private RETURN_TYPE lastVal;
        private long lastGet;
        private long delay;

        public Button(RETURN_TYPE btn) {
            this.btn = btn;
        }

        public RETURN_TYPE value() {
            if(System.currentTimeMillis() > lastGet + delay) {
                return this.btn;
            }
            else {
                return lastVal;
            }
        }

        public abstract RETURN_TYPE inverse();

        public void setDelay(long delay) {
            this.delay = delay;
        }
    }

    /*
     * Buttons that produce analogue values
     */
    public class AnalogueButton extends Button<Float>{
        public  AnalogueButton(float btn) {
            super(btn);
        }

        public float maxValue = 1;
        public float minValue = -1;

        public Float inverse() {
            return -1 * this.value();
        }
    }

    /*
     * Buttons that only produce to values
     */
    public class BinaryButton extends  Button<Boolean> {
        public  BinaryButton(boolean btn) {
            super(btn);
        }

        public Boolean inverse() {
            return ! this.value();
        }
    }
}
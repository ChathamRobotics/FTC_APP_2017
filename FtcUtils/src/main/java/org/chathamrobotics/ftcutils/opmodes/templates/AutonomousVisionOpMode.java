package org.chathamrobotics.ftcutils.opmodes.templates;

import org.chathamrobotics.ftcutils.Robot;
import org.chathamrobotics.ftcutils.StoppedException;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.detection.objects.Rectangle;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Point;
import org.opencv.core.Size;

/**
 * autonomous template for use with FTCVision
 */

public abstract class AutonomousVisionOpMode extends LinearVisionOpMode {
//    CONSTANTS         //
    private final static Size defaultFrameSize = new Size(900, 900);
    private final static ScreenOrientation defaultOrientation = ScreenOrientation.LANDSCAPE;

//    COMPONENTS        //

//    Stateful          //
    /*
     * Whether the current team is red
     */
    public boolean isRedTeam;


//    ABSTRACT METHODS  //
    abstract public Robot robot();

    /*
     * Called on start
     */
    abstract public void runRobot() throws StoppedException, InterruptedException;

//    METHODS           //
    /*
     * Initializes robot
     */
    public void initRobot() {
        robot();

        // Set to front facing camera
        this.setCamera(Cameras.PRIMARY);

        // Set frame size
        this.setFrameSize(defaultFrameSize);

        // Enable extensions
        enableExtension(Extensions.BEACON);
        enableExtension(Extensions.ROTATION);
        enableExtension(Extensions.CAMERA_CONTROL);

        // Set beacon analysis method
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        // Set color tolerance
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

//        beacon.setAnalysisBounds(new Rectangle(new Point(width / 2, height / 2), width - 200, 200));

        // Set rotation settings
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(defaultOrientation);

        // Camera controls
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
    }

    /*
     * called on stop
     */
    public void stopRobot() {
        this.robot().stop();
    }

    /*
     * Runs OpMode. Duh!
     */
    @Override
    public void runOpMode() throws InterruptedException {
        waitForVisionStart();

        initRobot();

        // Wait for start call
        waitForStart();

        debug();

        try {
            runRobot();
        }
        catch (StoppedException error) {
            //Just continue to robot stop
        }
        finally {
            debug();
            stopRobot();
        }
    }

    /*
     * periodically checks for stop and updates telemetry
     */
    public void statusCheck() throws StoppedException {
        debug();
        checkForStop();
    }

    /*
     * Updates telemetry readings
     */
    public void debug() {
        telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
        telemetry.addData("Beacon Center", beacon.getAnalysis().getLocationString());
        telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
        telemetry.addData("Beacon Buttons", beacon.getAnalysis().getButtonString());
        telemetry.addData("Screen Rotation", rotation.getScreenOrientationActual());
        telemetry.addData("Frame Rate", fps.getFPSString() + " FPS");
        telemetry.addData("Frame Size", "Width: " + width + " Height: " + height);

        this.robot().debug(false, true, true);

        telemetry.update();
    }

    /*
     * Checks if opmode is still active and if it's not throws a StoppedException
     */
    public void checkForStop() throws StoppedException{
        if (! opModeIsActive()) throw new StoppedException();
    }
}

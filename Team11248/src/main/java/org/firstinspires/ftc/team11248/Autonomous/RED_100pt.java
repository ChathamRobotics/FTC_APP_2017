package org.firstinspires.ftc.team11248.Autonomous;

import android.os.PowerManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team11248.Robot11248;

/**
 * blue autonomous 100 POINTS
 */
@Autonomous(name = "100ptRED")
public class RED_100pt extends LinearOpMode {

    Robot11248 robot;

    final int SONAR_DIST = 14;
    final int STOP_DELAY = 500;
    int A_SHOOT_TO_BEACON = 38;
    double rotationRatio = .004 ;
    int state = -1;


    int TIME_TO_FIRST_COLOR = 250;
    int TIME_TO_OTHER_COLOR = 500;
    int TIME_FORWARD_TO_BEACON = 550;

    final int FLAT = 180;

    @Override
    public void runOpMode() throws InterruptedException {

        //STAYS HERE UNTIL INIT BUTTON
        robot = new Robot11248(hardwareMap, telemetry);
        robot.init(); //Sets servos to right position.
        robot.activateColorSensors();
        robot.calibrateGyro(); //SETS ANGLE TOO 0 (BEFORE ANY MOVEMENT)
        robot.silent = false;

        waitForStart(); //STAYS HERE UNTIL PLAY BUTTON

        while (opModeIsActive() && !isStopRequested()) {
            //BEGIN AUTONOMOUS
            telemetry.addData("isBlue: ", robot.isBeaconBlue());
            telemetry.addData("isRed: ", robot.isBeaconRed());
            telemetry.addData("Sonar: ", robot.getSonarValue());
            telemetry.addData("ODS: ", robot.getLineSensorValue());
            telemetry.addData("Heading: ", robot.getGyroAngle());
            telemetry.addData("State: ", state);
            telemetry.update();

            switch (state) {
                case -1: //Forward and shoot
                    shootBallsStart(); //MOVES FORWARD AND SHOOT BALLS
                    sleep(1000);
                    robot.driveold(-.6, .6, 0);
                    sleep(1600);
                    state++;
                    break;

                case 0:
                    if(robot.moveToAngle(FLAT)) { //TODO: Why 173 degrees
                        robot.stop();
                        state++;
                        sleep(200);
                    }
                    break;

                case 1:
                    if(robot.moveToAngle(FLAT)) {
                        robot.stop();
                        state++;
                        sleep(200);
                    }
                    break;

                case 2: //Drive diagonal to line
                    // DRIVE DIAGONAL
                    robot.driveold(.3, -.3, 0);
                    if(robot.hitLine()) { //WHEN WHITE LINE FOUND
                        robot.stop(); //STOP MOVING
                        sleep(STOP_DELAY);
                        state++; //NEXT STATE
                    }
                    break;

                case 3: //adjust angle
                    if(robot.moveToAngle(FLAT)){
                        state++;
                        sleep(100);
                    }
                    break;

                case 4:
                    if(robot.getSonarValue() < SONAR_DIST+1) { // if too close, drive back
                        robot.driveold(.3, 0, 0);
                        state++;
                    }

                    else if(robot.getSonarValue() > SONAR_DIST-1) {// if too far, drive forward
                        robot.driveold(-.3, 0, 0);
                        state++;
                    }
                    else state+=2; // if good distance, no adjustment

                case 5:
                    //Y ADJUSTMENT
                    if (robot.getSonarValue() < SONAR_DIST+1 || robot.getSonarValue() > SONAR_DIST-1){ //stop when in tolerance
                        robot.stop();
                        sleep(STOP_DELAY);
                        state++;
                    }
                    break;

                case 6: //adjust angle
                    if(robot.moveToAngle(FLAT)) state++;
                    break;

                case 7: //Adjust x (hit if blue on left)
                    //X ADJUSTMENT
                    robot.driveold(0, -.35, 0);
                    if (robot.isBeaconRed()) {//WHEN BEACON IS BLUE
                        robot.driveold(0, -.35, 0);
                        sleep(90);
                        robot.stop();
                        sleep(500);
                        pushBeacon();
                        robot.stop();
                        sleep(500);
                        state += 2;
                    }
                    else if(robot.isBeaconBlue()) {
                        robot.stop();
                        state++;
                    }
                    break;

                case 8: //Adjust x (hit if red on right)
                    robot.driveold(0, -.35, 0); //MOVE LEFT
                    if(robot.isBeaconRed()) {
                        robot.driveold(0, -.35, 0);
                        sleep(200);
                        robot.stop(); //STOP MOVING
                        sleep(500);
                        pushBeacon();
                        state++;
                    }
                    break;

                case 9: //Backup and drive towards other line
                    robot.driveold(-.3, 0, 0);
                    sleep(1400);
                    robot.stop();
                    sleep(200);
                    robot.driveold(0, -.8, 0);
                    sleep(1400);
                    state++;
                    break;

                case 10: //keep driving until line hit
                    //telemetry.addData("y", xAgainstWall(SONAR_DIST));
                    robot.driveold(xAgainstWall(SONAR_DIST+2), -.37, 0);
                    if(robot.hitLine()) { //WHEN WHITE LINE FOUND
                        robot.stop(); //STOP MOVING
                        sleep(STOP_DELAY);
                        state++; //NEXT STATE
                    }
                    break;

                case 11: //adjust angle to 0
                    if(robot.moveToAngle(FLAT)) state++;
                    break;

                case 12:
                    if(robot.getSonarValue() < SONAR_DIST+1) { // if too close, drive back
                        robot.driveold(.3, 0, 0);
                        state++;
                    }

                    else if(robot.getSonarValue() > SONAR_DIST-1) {// if too far, drive forward
                        robot.driveold(-.3, 0, 0);
                        state++;
                    }
                    else state+=2; // if good distance, no adjustment

                case 13:
                    //Y ADJUSTMENT
                    if (robot.getSonarValue() < SONAR_DIST+1 || robot.getSonarValue() > SONAR_DIST-1){ //stop when in tolerance
                        robot.stop();
                        sleep(STOP_DELAY);
                        state++;
                    }
                    break;

                case 14: //adjust angle to 0
                    if(robot.moveToAngle(FLAT)) state++;
                    break;

                case 15: //Adjust x
                    //X ADJUSTMENT
                    robot.driveold(0, -.35, 0);
                    if (robot.isBeaconRed()) { //WHEN BEACON IS BLUE
                        robot.driveold(0, -.35, 0);
                        sleep(300);
                        robot.stop();
                        sleep(500);
                        pushBeacon();
                        robot.stop();
                        sleep(500);
                        state += 2;
                    }
                    else if(robot.isBeaconBlue()) {
                        robot.stop();
                        state++;
                    }
                    break;

                case 16: //adjust x
                    robot.driveold(0, -.35, 0); //MOVE LEFT .5
                    if(robot.isBeaconRed()) {
                        robot.driveold(0, -.35, 0);
                        sleep(400);
                        robot.stop(); //STOP MOVING
                        sleep(500);
                        pushBeacon();
                        state++;
                    }
                    break;

                default: //die
                    robot.stop();
                    idle();
                    break;
            }
        }
    }

    public void shootBallsStart(){
        robot.driveold(0,.8,0);
        sleep(500);

        //drive(0,0);
        robot.stop();
        sleep(500);

        robot.openCollector();
        robot.bangBang(.8f);
        sleep(750);

        robot.setConveyor(.2f);
        sleep(2000);

        robot.conveyorOff();
        robot.shooterOff();
        robot.closeCollector();

        //4.95 seconds
    }

    public void pushBeacon() {
        robot.moveBeaconOut(); //PUSH BEACON
        sleep(1000);
        robot.moveBeaconIn();
    }

    public void retrieveBeacon(long x, double speed) {
        //X ADJUSTMENT
        robot.driveold(0, -speed, 0);
        sleep(x);
        robot.stop();
        sleep(STOP_DELAY);

        if (robot.isBeaconBlue())//WHEN BEACON IS BLUE
            pushBeacon();
        else if (robot.isBeaconRed()) { //BEACON IS NOT BLUE (AKA ITS RED)
            robot.driveWithGyro(0, -speed, 0); //MOVE LEFT .5
            //sleep(TIME_TO_OTHER_COLOR); //WAIT .5 SECONDS
            robot.stop(); //STOP MOVING

            pushBeacon();
        }
    }

    public void driveAgainstWall(double speed, int angle, int distance){
        robot.driveWithGyro( xAgainstWall(distance) , speed, angle);
    }

    public double xAgainstWall(int distance){
        int SONAR_THRESHOLD = 2;
        double netDist = robot.getSonarValue() - distance;
        double y =0;

        if(Math.abs(netDist)> SONAR_THRESHOLD) {
            y = Math.abs(netDist) * .002 + .10;
            if(netDist<0) y*=-1;
        }

        return y;

    }
}

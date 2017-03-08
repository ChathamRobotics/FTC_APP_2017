package org.firstinspires.ftc.team11248.Autonomous.gyro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team11248.Robot11248;

/**
 * Generic Autonomous Gyro
 */
@Autonomous(name = "ERROR")
@Disabled
public class GENERIC_GYRO extends LinearOpMode {

    Robot11248 robot;

    public static final int SONAR_TOL = 1;

    public static final int STOP_DELAY = 370;
    public static final int BEACON_STOP = 500;

    int SONAR_DIST = 14;
    int state = 0;
    int FLAT = 0;

    @Override
    public void runOpMode() throws InterruptedException {}

    public void shootBallsStart(){
        robot.driveWithGyro(0,.8,FLAT);
        sleep(500);
        robot.stop();
        sleep(500);

        robot.openCollector();
        robot.setShooter(Robot11248.AUTO_SHOOTER_SPEED);
        sleep(750);

        robot.setConveyor(.2f);
        sleep(1150);
        robot.setConveyor(.8f);
        sleep(850);

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

    public void initAutonomous() {
        //STAYS HERE UNTIL INIT BUTTON
        robot = new Robot11248(hardwareMap, telemetry);
        robot.calibrateGyro(); //SETS ANGLE TOO 0 (BEFORE ANY MOVEMENT)
        robot.init(); //Sets servos to right position.
        robot.activateColorSensors();
        robot.silent = false;

        robot.deactivateServos();
        waitForStart(); //STAYS HERE UNTIL PLAY BUTTON
        robot.activateServos();
    }

    //Case 0, drives forward then shoots
    public void forwardShoot() {
        shootBallsStart(); //MOVES FORWARD AND SHOOT BALLS
        sleep(1000);
        robot.driveWithGyro(.6, .6, FLAT);
        sleep(1600);
        state++;
    }

    //CASE 1, drives diagonally to first line
    public void diagonalLine() {
        robot.driveWithGyro(.3, .3, FLAT); //DRIVE DIAGONAL
        if(robot.hitLine()) { //WHEN WHITE LINE FOUND
            robot.stop(); //STOP MOVING
            sleep(STOP_DELAY);
            state++; //NEXT STATE
        }
    }

    //CASE 2 & 9, adjusts distance to wall to be in right range
    public void adjustWallDistance() {
        if (robot.getSonarValue() <= SONAR_DIST + SONAR_TOL &&
                robot.getSonarValue() >= SONAR_DIST - SONAR_TOL) {
            sleep(STOP_DELAY);
            state++;
        }

        // if too close, drive back
        else if(robot.getSonarValue() < SONAR_DIST - SONAR_TOL)
            robot.driveWithGyro(-.3, 0, FLAT);

            //if too far, drive forward
        else if(robot.getSonarValue() > SONAR_DIST + SONAR_TOL)
            robot.driveWithGyro(.3, 0, FLAT);

    }

    //CASE 5, backs up and starts to drive to second line
    public void backupDrive() {
        robot.driveWithGyro(-.3, 0, FLAT);
        sleep(1400); //back up a bit (kinda arbitrary)
        robot.driveold(0, .8, FLAT);
        sleep(1390); //Drive fast for a while to cut on time
        state++;
    }

    //CASE 6, Drives to second line
    public void driveToLine2(boolean isBlue) {
        if(isBlue)
            robot.driveWithGyro(xAgainstWall(SONAR_DIST+3), .35, FLAT);
        else
            robot.driveWithGyro(xAgainstWall(SONAR_DIST+3), -.35, FLAT);

        if(robot.hitLine()) {
            robot.stop();
            sleep(STOP_DELAY);
            state++;
        }
    }

    //CASE 10, Drives diagonally to cap ball and parks.
    public void driveToCap() {
        robot.driveWithGyro(-1, -1, FLAT); //drive to cap ball
        sleep(2750);
        state++;
    }

    //UTIL Methods

    public double xAgainstWall(int distance){

        int SONAR_THRESHOLD = 2;
        double netDist = robot.getSonarValue() - distance;
        double y = 0;

        if(Math.abs(netDist)> SONAR_THRESHOLD) {
            y = Math.abs(netDist) * .002 + .085;
            if(netDist<0) y*=-1;
        }
        return y;
    }

    public void doTelemetry() {
        telemetry.addData("isBlue", robot.isBeaconBlue());
        telemetry.addData("isRed", robot.isBeaconRed());
        telemetry.addData("Sonar", robot.getSonarValue());
        telemetry.addData("ODS: ", robot.getLineSensorValue());
        telemetry.addData("Heading: ", robot.getGyroAngle());
        telemetry.addData("Flat: ", FLAT);
        telemetry.addData("State: ", state);
        telemetry.update();
    }
}

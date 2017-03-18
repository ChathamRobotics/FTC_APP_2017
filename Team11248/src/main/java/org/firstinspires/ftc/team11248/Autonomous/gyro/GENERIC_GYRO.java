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
    public static final double SLOW_SPEED = .35;

    int SONAR_DIST = 12;
    int state = 0;
    int FLAT = 0;

    public double x, y;

    public static boolean isBlue;

    private boolean rightSide = true;


    @Override
    public void runOpMode() throws InterruptedException {

        initAutonomous();

        while (opModeIsActive() && !isStopRequested()) {
            //BEGIN AUTONOMOUS
            doTelemetry();

            switch (state) {
                case 0: //Forward and shoot
                    forwardShoot();
                    break;

                case 1: //Drive diagonal to line
                    diagonalLine();
                    break;

                case 2: // Y ADJUSTMENT
                    adjustWallDistance();
                    break;

                case 3:
                    retrieveBeacon();
                    break;

                case 4: //Backup and drive towards other line
                    backupDrive();
                    break;

                case 5: //keep driving until line hit
                    driveToLine2();
                    break;

                case 6:
                    //Y ADJUSTMENT
                    adjustWallDistance();
                    rightSide = true;
                    break;

                case 7: //Adjust x (hit if blue on left)
                    retrieveBeacon();
                    break;

                case 8:
                    driveToCap();
                    break;

                default: //die
                    drive(0,0);
                    idle();
                    break;
            }

            robot.driveWithGyro(x, y, FLAT);
        }
    }

    public void shootBallsStart(){

        drive(0,.8);
        sleep(500);
        drive(0, 0);

        robot.openCollector();
        robot.setShooter(Robot11248.AUTO_SHOOTER_SPEED);
        sleep(1500);

        robot.setConveyor(.6f);
        sleep(1500);

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

        FLAT = robot.getGyroAngle();
        shootBallsStart(); //MOVES FORWARD AND SHOOT BALLS

        FLAT = isBlue? FLAT: (180 + FLAT) % 360;
        sleep(1500);
        drive(.6, isBlue?.6:-.6);
        sleep(1600);
        state++;
    }

    //CASE 1, drives diagonally to first line
    public void diagonalLine() {
        drive(.25, isBlue?.25:-.25); //DRIVE DIAGONAL
        if(robot.hitLine()) { //WHEN WHITE LINE FOUND
            drive(0,0); //STOP MOVING
            sleep(STOP_DELAY);
            state++; //NEXT STATE
        }
    }

    //CASE 2 & 6, adjusts distance to wall to be in right range
    public void adjustWallDistance() {
        if (robot.getSonarValue() <= SONAR_DIST + SONAR_TOL &&
                robot.getSonarValue() >= SONAR_DIST - SONAR_TOL) {
            drive(0, 0);
            sleep(STOP_DELAY);
            state++;
        }

        // if too close, drive back
        else if(robot.getSonarValue() < SONAR_DIST - SONAR_TOL) drive(-.3, 0);

            //if too far, drive forward
        else if(robot.getSonarValue() > SONAR_DIST + SONAR_TOL) drive(.3, 0);

    }

    //CASE 3, 7 senses navigates and presses correct button
    public void retrieveBeacon(){

            if(rightSide) {//Adjust x (hit if blue on left)
                //WHEN BEACON IS good color
                if (isBlue ? robot.isBeaconBlue() : robot.isBeaconRed()) {
                    pushBeacon();
                    state++; //Skip to state 5
                }

                //WHEN BEACON IS LEFT
                else rightSide = false;

            }else{

                drive(0, -SLOW_SPEED);
                if (robot.hitLine2()) {
                    //sleep(300);
                    drive(0, 0);
                    sleep(500);
                    pushBeacon();
                    state++;
                }

        }
    }

    //CASE 4, backs up and starts to drive to second line
    public void backupDrive() {
        drive(-.3, 0);
        sleep(1400); //back up a bit (kinda arbitrary)
        drive(0, isBlue?.8:-.8);
        sleep(1390); //Drive fast for a while to cut on time
        state++;
    }

    //CASE 5, Drives to second line
    public void driveToLine2() {

        drive(xAgainstWall(SONAR_DIST+3), isBlue?SLOW_SPEED:-SLOW_SPEED);

        if(robot.hitLine()){
            drive(0,0);
            sleep(STOP_DELAY);
            state++;
        }
    }

    //CASE 8, Drives diagonally to cap ball and parks.
    public void driveToCap() {
        drive (-1, isBlue? -1:1); //drive to cap ball
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

    public void drive(double x, double y){
        this.x = x;
        this.y = y;

        robot.driveWithGyro(x, y, FLAT);
    }

    public void sleep (int millis){
        long start = System.currentTimeMillis();
        while(System.currentTimeMillis() - start < millis && opModeIsActive()){
            robot.driveWithGyro(x, y, FLAT);
        }
    }

}

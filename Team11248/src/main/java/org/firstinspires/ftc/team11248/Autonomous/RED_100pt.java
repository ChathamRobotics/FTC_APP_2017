package org.firstinspires.ftc.team11248.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team11248.Robot11248;

/**
 * red autonomous 100 points
 */
@Autonomous(name = "100ptRED")
public class RED_100pt extends LinearOpMode {

    Robot11248 robot;

    final int STOP_DELAY = 500;
    int A_SHOOT_TO_BEACON = 38;
    double rotationRatio = .004 ;
    int state = -1;


    int TIME_TO_FIRST_COLOR = 250;
    int TIME_TO_OTHER_COLOR = 500;
    int TIME_FORWARD_TO_BEACON = 550;
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
            telemetry.addData("isBlue", robot.isBeaconBlue());
            telemetry.addData("isRed", robot.isBeaconRed());
            telemetry.addData("sonar", robot.getSonarValue());
            telemetry.update();

            switch (state) {
                case -1: //Forward and shoot
                    shootBallsStart(); //MOVES FORWARD AND SHOOT BALLS
                    sleep(1000);
                    state++;
                    break;
                case 0: //Drive diagonal to line
                    robot.driveold(.3, .3, 0); //DRIVE DIAGONAL
                    if(robot.hitLine()) { //WHEN WHITE LINE FOUND
                        robot.stop(); //STOP MOVING
                        sleep(STOP_DELAY);
                        state++; //NEXT STATE
                    }
                    break;
                case 1: //adjust angle
                    if(robot.moveToAngle(0)) state++;
                    break;
                case 2: //adjust y (closeness to wall)
                    //Y ADJUSTMENT
                    robot.driveold(.3, 0, 0);
                    if (robot.getSonarValue() < 12){
                        robot.stop();
                        sleep(STOP_DELAY);
                        state++;
                    }
                    break;
                case 3: //Adjust x (hit if red on left)
                    //X ADJUSTMENT
                    robot.driveold(0, -.35, 0);
                    if (robot.isBeaconRed()) {//WHEN BEACON IS BLUE
                        robot.driveold(0, -.35, 0);
                        sleep(300);
                        robot.stop();
                        sleep(1000);
                        pushBeacon();
                        robot.stop();
                        sleep(1000);
                        state += 2;
                    }
                    else if(robot.isBeaconBlue())
                        state++;
                    break;
                case 4: //Adjust x (hit if blue on right)
                    robot.driveold(0, -.35, 0); //MOVE LEFT
                    if(robot.isBeaconRed()) {
                        robot.driveold(0, -.35, 0);
                        sleep(400);
                        robot.stop(); //STOP MOVING
                        sleep(1000);
                        pushBeacon();
                        state++;
                    }
                    break;
                case 5: //Backup and drive towards other line
                    robot.driveold(-.3, 0, 0);
                    sleep(1000);
                    robot.stop();
                    sleep(200);
                    robot.driveold(0, .3, 0); //DRIVE DIAGONAL
                    sleep(2000);
                    state++;
                    break;
                case 6: //keep driving until line hit
                    robot.driveold(0, .3, 0);
                    if(robot.hitLine()) { //WHEN WHITE LINE FOUND
                        robot.stop(); //STOP MOVING
                        sleep(STOP_DELAY);
                        state++; //NEXT STATE
                    }
                case 7: //adjust angle to 0
                    if(robot.moveToAngle(0)) state++;
                    break;
                case 8: //Adjust y (closeness to wall)
                    //Y ADJUSTMENT
                    robot.driveold(.3, 0, 0);
                    if (robot.getSonarValue() < 12){
                        robot.stop();
                        sleep(STOP_DELAY);
                        state++;
                    }
                    break;
                case 9: //Adjust x
                    //X ADJUSTMENT
                    robot.driveold(0, .35, 0);
                    if (robot.isBeaconRed()) {//WHEN BEACON IS BLUE
                        robot.driveold(0, .35, 0);
                        sleep(300);
                        robot.stop();
                        sleep(1000);
                        pushBeacon();
                        robot.stop();
                        sleep(1000);
                        state += 2;
                    }
                    else if(robot.isBeaconBlue())
                        state++;
                    break;
                case 10: //adjust x
                    robot.driveold(0, .35, 0); //MOVE LEFT .5
                    if(robot.isBeaconRed()) {
                        robot.driveold(0, .35, 0);
                        sleep(400);
                        robot.stop(); //STOP MOVING
                        sleep(1000);
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
        robot.driveWithGyro(0,.8,0);
        sleep(500);

        //drive(0,0);
        robot.stop();
        sleep(500);

        robot.openCollector();
        robot.shooterOn();
        sleep(750);

        robot.setConveyor(.2f);
        sleep(2500);

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
        int SONAR_THRESHOLD = 5;
        double netDist = robot.getSonarValue() - distance;
        double y =0;

        if(Math.abs(netDist)> SONAR_THRESHOLD) {
            y = Math.abs(netDist) * .003 + .25;
            if(netDist<0) y*=-1;
        }
        robot.driveWithGyro( y , speed, angle);
    }
//    public void driveWithGyro(double x, double y, int targetAngle){
//
//        int currentAngle = robot.getGyroAngle();
//        int net = currentAngle - targetAngle;
//        double rotation = -.25;
//
//        if(net > 180) { // if passes 0
//            if(currentAngle > 180) //counterclockwise past 0
//                net = (360 - currentAngle) + targetAngle;
//
//            else
//                net = (targetAngle - 360) + currentAngle;
//        }
//
//        //rotation = -.25;//net * rotationRatio + .25;
//
//        telemetry.addData("1:", "Heading: " + robot.getGyroAngle());
//        telemetry.addData("2:", "Net: " + net);
//        telemetry.addData("3: ", "Speed: " +rotation);
//        telemetry.addData("4: ",  "Target: " + targetAngle);
//
//        telemetry.update();
//
//        robot.driveold(x,y,rotation,false);
//    }
}

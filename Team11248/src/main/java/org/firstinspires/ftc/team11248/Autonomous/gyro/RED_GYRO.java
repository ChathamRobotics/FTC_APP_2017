package org.firstinspires.ftc.team11248.Autonomous.gyro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team11248.Robot11248;

/**
 * red autonomous 100 POINTS
 */
@Autonomous(name = "REDGyro")
public class RED_GYRO extends GENERIC_GYRO {

    Robot11248 robot;

    double x, y;
    int SONAR_DIST = 13;

    @Override
    public void runOpMode() throws InterruptedException {
        initAutonomous();

        while (opModeIsActive() && !isStopRequested()) {
            //BEGIN AUTONOMOUS
            doTelemetry();

            switch (state) {
                case 0: //Forward and shoot
                    FLAT = (180 + robot.getGyroAngle()) % 360;
                    forwardShoot();
                    break;

                case 1: //Drive diagonal to line
                    diagonalLine();
                    break;

                case 2: // Y ADJUSTMENT
                    adjustWallDistance();
                    break;

                case 3: //Adjust x (hit if blue on left)
                    //X ADJUSTMENT
                    robot.driveWithGyro(0, -.35, FLAT);
                    if (robot.isBeaconRed()) {//WHEN BEACON IS BLUE
                        robot.driveWithGyro(0, -.35, FLAT);
                        sleep(400);
                        robot.stop();
                        sleep(BEACON_STOP);
                        pushBeacon();
                        robot.stop();
                        sleep(BEACON_STOP);
                        state += 2;
                    }
                    else if(robot.isBeaconBlue())
                        state++;
                    break;

                case 4: //Adjust x (hit if red on right)
                    robot.driveWithGyro(0, -.35, FLAT); //MOVE LEFT
                    if(robot.isBeaconRed()) {
                        robot.driveWithGyro(0, -.35, FLAT);
                        sleep(150);
                        robot.stop(); //STOP MOVING
                        sleep(BEACON_STOP);
                        pushBeacon();
                        state++;
                    }
                    break;

                case 5: //Backup and drive towards other line
                    backupDrive();
                    break;

                case 6: //keep driving until line hit
                    driveToLine2(false);
                    break;

                case 7:
                    //Y ADJUSTMENT
                    adjustWallDistance();
                    break;

                case 8: //Adjust x
                    //X ADJUSTMENT
                    robot.driveold(0, -.35, 0);
                    if (robot.isBeaconRed()) { //WHEN BEACON IS BLUE
                        robot.driveold(0, -.35, 0);
                        sleep(400);
                        robot.stop();
                        sleep(BEACON_STOP);
                        pushBeacon();
                        robot.stop();
                        sleep(BEACON_STOP);
                        state += 2;
                    }
                    else if(robot.isBeaconBlue())
                        state++;
                    break;

                case 9: //adjust x
                    robot.driveWithGyro(0, -.35, FLAT); //MOVE LEFT .5
                    if(robot.isBeaconRed()) {
                        robot.driveold(0, -.35, 0);
                        sleep(200);
                        robot.stop(); //STOP MOVING
                        sleep(BEACON_STOP);
                        pushBeacon();
                        state++;
                    }
                    break;

                case 10:
                    driveToCap();
                    break;

                default: //die
                    robot.stop();
                    idle();
                    break;
            }
        }
    }
}

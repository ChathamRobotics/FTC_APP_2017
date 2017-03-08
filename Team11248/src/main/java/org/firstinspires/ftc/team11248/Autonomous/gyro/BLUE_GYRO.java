package org.firstinspires.ftc.team11248.Autonomous.gyro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team11248.Robot11248;

/**
 * blue autonomous 100 POINTS
 */
@Autonomous(name = "BLUEGyro")
public class BLUE_GYRO extends GENERIC_GYRO {

    Robot11248 robot;

    @Override
    public void runOpMode() throws InterruptedException {
        initAutonomous();

        //BEGIN AUTONOMOUS
        while (opModeIsActive() && !isStopRequested()) {
            doTelemetry();

            switch (state) {
                case 0: //Forward and shoot
                    FLAT = robot.getGyroAngle();
                    forwardShoot();
                    break;

                case 1: //Drive diagonal to line
                    diagonalLine();
                    break;

                case 2: //adjust closeness to wall
                    adjustWallDistance();
                    break;

                case 3: //Adjust x (hit if blue on left)
                    //X ADJUSTMENT
                    robot.driveWithGyro(0, -.35, FLAT);

                    //WHEN BEACON IS BLUE
                    if (robot.isBeaconBlue()) {
                        robot.driveWithGyro(0, -.35, FLAT);
                        sleep(400); //TODO MANUAL TIME
                        robot.stop();
                        sleep(BEACON_STOP);
                        pushBeacon();
                        sleep(BEACON_STOP);
                        state += 2; //Skip to state 5
                    }
                    //WHEN BEACON IS RED LEFT
                    else if(robot.isBeaconRed())
                        state++;

                    break;

                case 4: //Adjust x (hit if red on right)
                    robot.driveWithGyro(0, -.35, FLAT); //MOVE LEFT
                    if(robot.isBeaconBlue()) {
                        robot.driveold(0, -.35, 0);
                        sleep(400); //TODO MANUAL TIME
                        robot.stop();
                        sleep(BEACON_STOP);
                        pushBeacon();
                        state++;
                    }
                    break;

                case 5: //Backup and drive towards other line
                    backupDrive();
                    break;

                case 6: //keep driving until line hit
                    driveToLine2(true);
                    break;

                case 7: //Adjust y (closeness to wall)
                    adjustWallDistance();
                    break;

                case 8: //Adjust x
                    //X ADJUSTMENT
                    robot.driveWithGyro(0, -.35, FLAT);
                    //WHEN BEACON IS BLUE
                    if (robot.isBeaconBlue()) {
                        robot.driveWithGyro(0, -.35, FLAT);
                        sleep(300); //TODO MANUAL TIME
                        robot.stop();
                        sleep(BEACON_STOP);
                        pushBeacon();
                        robot.stop();
                        sleep(BEACON_STOP);
                        state += 2; //skip to state 13
                    }
                    else if(robot.isBeaconRed())
                        state++;
                    break;

                case 9: //Adjust x (hit if blue on right)
                    robot.driveWithGyro(0, -.35, FLAT); //MOVE LEFT .5
                    if(robot.isBeaconBlue()) {
                        robot.driveWithGyro(0, -.35, FLAT);
                        sleep(200); //TODO MANUAL TIME
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

package org.firstinspires.ftc.team11248.Autonomous.gyro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team11248.Robot11248;

/**
 * blue autonomous 100 POINTS
 */
@Autonomous(name = "BLUEGyro")
public class BLUE_GYRO extends LinearOpMode {

    Robot11248 robot = new Robot11248(hardwareMap, telemetry);
    GENERIC_GYRO template = new GENERIC_GYRO();

    @Override
    public void runOpMode() throws InterruptedException {

        template.initAutonomous();

        //BEGIN AUTONOMOUS
        while (opModeIsActive() && !isStopRequested()) {
            template.doTelemetry();

            switch (template.state) {
                case 0: //Forward and shoot
                    template.FLAT = robot.getGyroAngle();
                    template.forwardShoot();
                    break;

                case 1: //Drive diagonal to line
                    template.diagonalLine();
                    break;

                case 2: //adjust closeness to wall
                    template.adjustWallDistance();
                    break;

                case 3: //Adjust x (hit if blue on left)
                    //X ADJUSTMENT
                    robot.driveWithGyro(0, -.35, template.FLAT);

                    //WHEN BEACON IS BLUE
                    if (robot.isBeaconBlue()) {
                        robot.driveWithGyro(0, -.35, template.FLAT);
                        sleep(400); //TODO MANUAL TIME
                        robot.stop();
                        sleep(template.BEACON_STOP);
                        template.pushBeacon();
                        sleep(template.BEACON_STOP);
                        template.state += 2; //Skip to state 5
                    }
                    //WHEN BEACON IS RED LEFT
                    else if(robot.isBeaconRed())
                        template.state++;

                    break;

                case 4: //Adjust x (hit if red on right)
                    robot.driveWithGyro(0, -.35, template.FLAT); //MOVE LEFT
                    if(robot.isBeaconBlue()) {
                        robot.driveold(0, -.35, 0);
                        sleep(400); //TODO MANUAL TIME
                        robot.stop();
                        sleep(template.BEACON_STOP);
                        template.pushBeacon();
                        template.state++;
                    }
                    break;

                case 5: //Backup and drive towards other line
                    template.backupDrive();
                    break;

                case 6: //keep driving until line hit
                    template.driveToLine2();
                    break;

                case 7: //Adjust y (closeness to wall)
                    template.adjustWallDistance();
                    break;

                case 8: //Adjust x
                    //X ADJUSTMENT
                    robot.driveWithGyro(0, -.35, template.FLAT);
                    //WHEN BEACON IS BLUE
                    if (robot.isBeaconBlue()) {
                        robot.driveWithGyro(0, -.35, template.FLAT);
                        sleep(300); //TODO MANUAL TIME
                        robot.stop();
                        sleep(template.BEACON_STOP);
                        template.pushBeacon();
                        robot.stop();
                        sleep(template.BEACON_STOP);
                        template.state += 2; //skip to state 13
                    }
                    else if(robot.isBeaconRed())
                        template.state++;
                    break;

                case 9: //Adjust x (hit if blue on right)
                    robot.driveWithGyro(0, -.35, template.FLAT); //MOVE LEFT .5
                    if(robot.isBeaconBlue()) {
                        robot.driveWithGyro(0, -.35, template.FLAT);
                        sleep(200); //TODO MANUAL TIME
                        robot.stop(); //STOP MOVING
                        sleep(template.BEACON_STOP);
                        template.pushBeacon();
                        template.state++;
                    }
                    break;

                case 10:
                    template.driveToCap();
                    break;

                default: //die
                    robot.stop();
                    idle();
                    break;
            }
        }
    }
}

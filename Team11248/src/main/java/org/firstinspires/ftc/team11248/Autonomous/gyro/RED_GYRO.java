package org.firstinspires.ftc.team11248.Autonomous.gyro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team11248.Robot11248;

/**
 * red autonomous 100 POINTS
 */
@Autonomous(name = "REDGyro")
@Disabled
public class RED_GYRO extends LinearOpMode {

    Robot11248 robot = new Robot11248(hardwareMap, telemetry);
    GENERIC_GYRO template = new GENERIC_GYRO();

    @Override
    public void runOpMode() throws InterruptedException {

        template.initAutonomous();

        while (opModeIsActive() && !isStopRequested()) {
            //BEGIN AUTONOMOUS
            template.doTelemetry();

            switch (template.state) {
                case 0: //Forward and shoot
                    template.FLAT = (180 + robot.getGyroAngle()) % 360;
                    template.forwardShoot();
                    break;

                case 1: //Drive diagonal to line
                    template.diagonalLine();
                    break;

                case 2: // Y ADJUSTMENT
                    template.adjustWallDistance();
                    break;

                case 3: //Adjust x (hit if blue on left)
                    //X ADJUSTMENT
                    robot.driveWithGyro(0, -.35, template.FLAT);
                    if (robot.isBeaconRed()) {//WHEN BEACON IS BLUE
                        robot.driveWithGyro(0, -.35, template.FLAT);
                        sleep(400);
                        robot.stop();
                        sleep(template.BEACON_STOP);
                        template.pushBeacon();
                        robot.stop();
                        sleep(template.BEACON_STOP);
                        template.state += 2;
                    }
                    else if(robot.isBeaconBlue())
                        template.state++;
                    break;

                case 4: //Adjust x (hit if red on right)
                    robot.driveWithGyro(0, -.35, template.FLAT); //MOVE LEFT
                    if(robot.isBeaconRed()) {
                        robot.driveWithGyro(0, -.35, template.FLAT);
                        sleep(150);
                        robot.stop(); //STOP MOVING
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

                case 7:
                    //Y ADJUSTMENT
                    template.adjustWallDistance();
                    break;

                case 8: //Adjust x
                    //X ADJUSTMENT
                    robot.driveold(0, -.35, 0);
                    if (robot.isBeaconRed()) { //WHEN BEACON IS BLUE
                        robot.driveold(0, -.35, 0);
                        sleep(400);
                        robot.stop();
                        sleep(template.BEACON_STOP);
                        template.pushBeacon();
                        robot.stop();
                        sleep(template.BEACON_STOP);
                        template.state += 2;
                    }
                    else if(robot.isBeaconBlue())
                        template.state++;
                    break;

                case 9: //adjust x
                    robot.driveWithGyro(0, -.35, template.FLAT); //MOVE LEFT .5
                    if(robot.isBeaconRed()) {
                        robot.driveold(0, -.35, 0);
                        sleep(200);
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

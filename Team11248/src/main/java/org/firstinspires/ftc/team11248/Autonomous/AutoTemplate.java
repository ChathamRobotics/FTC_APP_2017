package org.firstinspires.ftc.team11248.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team11248.Robot11248;

/**
 * Created by Tony_Air on 2/3/17.
 */

public abstract class AutoTemplate extends LinearOpMode{

    public Robot11248 robot;

    public AutoTemplate(Robot11248 robot){
        this.robot = robot;
    }


    public void shootBallsStart(){
        robot.driveold(0,.8,0);
        sleep(500);

        //drive(0,0);
        robot.stop();
        sleep(500);

        robot.openCollector();
        robot.setShooter(.65f);
        sleep(750);

        robot.setConveyor(.2f);
        sleep(1000);
        robot.setConveyor(.8f);
        sleep(1000);

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

    public void driveAgainstWall(double speed, int angle, int distance){
        robot.driveWithGyro( xAgainstWall(distance) , speed, angle);
    }

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


}

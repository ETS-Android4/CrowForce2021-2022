package org.firstinspires.ftc.teamcode;

import com.SCHSRobotics.HAL9001.system.robot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.system.robot.MainRobot;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.CoordinateMode;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.HALTrajectory;
import com.SCHSRobotics.HAL9001.util.math.geometry.Point2D;
import com.SCHSRobotics.HAL9001.util.math.geometry.Vector2D;
import com.SCHSRobotics.HAL9001.util.math.units.HALDistanceUnit;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.PI;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystem.WeirdSpinnerHalWorkaround;

@Autonomous(name = "PIDTest", group = "Example Programs")
public class PIDTestAutonomous extends BaseAutonomous {
    public @MainRobot Baguette robot;
    WeirdSpinnerHalWorkaround weird = new WeirdSpinnerHalWorkaround();


    @Override
    public void main() {
        DcMotor flm = hardwareMap.dcMotor.get("f_l_m");
        DcMotor frm = hardwareMap.dcMotor.get("f_r_m");
        DcMotor blm = hardwareMap.dcMotor.get("b_l_m");
        DcMotor brm = hardwareMap.dcMotor.get("b_r_m");

        /*HALTrajectory forwardRoute = new HALTrajectory(robot.mDrive.trajectoryBuilder(new Pose2d(0,0), 0).

                lineTo(new Point2D(0,10000)).
                build().
                toRoadrunner(),
                CoordinateMode.HAL);

        HALTrajectory returnRoute = new HALTrajectory(robot.mDrive.trajectoryBuilder(forwardRoute.end(), 0).
                lineTo(new Point2D(0,0)).
                build().
                toRoadrunner(),
                CoordinateMode.HAL);*/


        weird.main();


        //robot.mDrive.moveSimple(new Vector2D(0,48), 1);
        waitTime(1000);
        robot.mDrive.turnPID(PI, PI/60);
        //telemetry.addData("thing:", "turning");
        //telemetry.update();
        /*waitTime(1000);
        robot.mDrive.turnPID(PI);
        waitTime(1000);
        robot.mDrive.turnPID(PI);
        */

        waitTime(1000);
        //telemetry.addData("thing:", "forwardHAL");
        //telemetry.update();
        robot.mDrive.moveSimple(new Vector2D(0,-48), 1);

        waitTime(5000);
        //telemetry.addData("thing:", "stopHAL");
        //telemetry.update();
        robot.mDrive.setMotorPower("f_r_m", 0);
        robot.mDrive.setMotorPower("b_r_m", 0);
        robot.mDrive.setMotorPower("f_l_m", 0);
        robot.mDrive.setMotorPower("b_l_m", 0);
        waitTime(3000);



    }
}

package org.firstinspires.ftc.teamcode.Autonomi;

import com.SCHSRobotics.HAL9001.system.robot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.system.robot.MainRobot;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.CoordinateMode;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.HALTrajectory;
import com.SCHSRobotics.HAL9001.util.math.geometry.Point2D;
import com.SCHSRobotics.HAL9001.util.math.geometry.Vector2D;
import com.SCHSRobotics.HAL9001.util.math.units.HALAngleUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALDistanceUnit;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.PI;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Baguette;
import org.firstinspires.ftc.teamcode.Subsystem.WeirdSpinnerHalWorkaround;

@Autonomous(name = "PIDTest", group = "Example Programs")
public class PIDTestAutonomous extends BaseAutonomous {
    public @MainRobot
    Baguette robot;
    //WeirdSpinnerHalWorkaround weird = new WeirdSpinnerHalWorkaround();


    @Override
    public void main() {
        robot.mDrive.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));

        robot.mDrive.turnPID(Math.PI/2);
        telemetry.addData("flag:", "1");
        telemetry.update();
        waitTime(1000);
        robot.mDrive.turnPID(270, HALAngleUnit.DEGREES);
        waitTime(1000);
        robot.mDrive.turnPID(180, HALAngleUnit.DEGREES);
        waitTime(1000);


    }
}

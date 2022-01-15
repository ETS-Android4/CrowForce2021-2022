package org.firstinspires.ftc.teamcode;

import com.SCHSRobotics.HAL9001.system.robot.ExternalCamera;
import com.SCHSRobotics.HAL9001.system.robot.InternalCamera;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.localizer.HolonomicDriveEncoderIMULocalizer;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.RoadrunnerConfig;
import com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain.DriveConfig;
import com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain.MecanumDrive;
import com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain.MecanumDriveSimple;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystem.Arm2;
import org.firstinspires.ftc.teamcode.Subsystem.ArmController;
import org.firstinspires.ftc.teamcode.Subsystem.DuckSpinner;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.VisionStuff.Vision;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class Baguette extends Robot {
    //@InternalCamera(resWidth = 320, resHeight = 240, usesViewport = true)
//  @ExternalCamera(resWidth = 320, resHeight = 240, configName = "webcam", usesViewport = false)
    //public OpenCvCamera camera = getCamera("webcam");

    public MecanumDrive mDrive;
    public DuckSpinner spinner;
    public ArmController arm;
    public Arm2 arm2;
    public Intake intake;
    //public Vision vision;
    public static boolean isRunningAuto = true;

    //@ExternalCamera(resHeight = 720, resWidth = 1080, configName = "Cam", uniqueId = "cam", usesViewport = true)
    //public OpenCvCamera camera = getCamera("cam");

    public Baguette(OpMode opMode) {
        super(opMode);
        //opMode.telemetry.clear();
        //this.telemetry.clear();

        mDrive = new MecanumDrive(
                this,
                //TODO: change these later
                //15.25 from edge to edge
                new RoadrunnerConfig(3, 27.4, 13.75,  384.5, 435),
                "f_l_m",
                "f_r_m",
                "b_l_m",
                "b_r_m",
                true
        );

        mDrive.setLocalizer(new HolonomicDriveEncoderIMULocalizer(
                this,
                mDrive,
                "imu",
                "f_l_m",
                "f_r_m",
                "b_l_m",
                "b_r_m"
        ));
        Localizer l = mDrive.getLocalizer();

        //l.setPoseEstimate(new Pose2d(l.getPoseEstimate().getX(), l.getPoseEstimate().getY(), l.getPoseEstimate().getHeading() + 180));

        mDrive.reverseMotor("f_l_m");
        mDrive.reverseMotor("f_r_m");
        mDrive.reverseMotor("b_l_m");
        mDrive.reverseMotor("b_r_m");

        mDrive.setTurnPID(new PIDCoefficients(0.7, 0, 0.0275));
        //mDrive.set
        mDrive.setHeadingPIDTolerance(Math.PI/45);
        //mDrive.setHeadingPID(new PIDCoefficients(0.5, 0, 0.05));
       // mDrive.setTranslationalPID(new PIDCoefficients(1, 0, 0.05));
        //arm = new ArmController(this, "big_s", "clamp_s");
        arm2 = new Arm2(this, "clamp_s", "big_s");
        spinner = new DuckSpinner(this, "spin_motor");
        intake = new Intake(this, "slide", "intake", "tray_servo");
        //vision = new Vision(this);
    }
}

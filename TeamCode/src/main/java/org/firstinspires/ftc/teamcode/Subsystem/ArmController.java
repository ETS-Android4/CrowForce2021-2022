package org.firstinspires.ftc.teamcode.Subsystem;

import com.SCHSRobotics.HAL9001.system.robot.MainRobot;
import com.SCHSRobotics.HAL9001.system.robot.SubSystem;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.MyRobot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

//@TeleOp(name = "Arm Controls", group = "robot")
public class ArmController extends SubSystem {

    public @MainRobot
    MyRobot robot;

    Servo elbowJoint;
    Servo clampServo;


    public ArmController(MyRobot robot, String BIG_S, String CLAMP_S){
        super(robot);
        elbowJoint = hardwareMap.servo.get(BIG_S);
        clampServo = hardwareMap.servo.get(CLAMP_S);
    }


    double armPos = 0.0;

    @Override
    public void init() {
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void handle() {
        double py = -gamepad1.left_stick_y;
        boolean down = gamepad1.dpad_down;
        boolean up = gamepad1.dpad_up;
        boolean grab = gamepad1.right_bumper;
        boolean letGo = gamepad1.left_bumper;
        boolean thirdLevel = gamepad1.triangle;
        boolean secondLevel = gamepad1.square;
        boolean firstLevel = gamepad1.x;

        armPos = elbowJoint.getPosition();

        while(py > 0.5)
        {
            elbowJoint.setPosition(armPos + 1);
            armPos += 1;
        }
        while(py < -0.5)
        {
            elbowJoint.setPosition(armPos - 1);
            armPos -= 1;
        }

        if(down == true)
        {
            elbowJoint.setPosition(0);
        }
        else if(up == true)
        {
            elbowJoint.setPosition(180);
        }

        if(grab == true)
        {
            clampServo.setPosition(80);
        }
        else if(letGo == true)
        {
            clampServo.setPosition(0);
        }

        if(firstLevel == true) {
            elbowJoint.setPosition(30);
        }
        else if(secondLevel == true)
        {
            elbowJoint.setPosition(45);
        }
        else if(thirdLevel == true)
        {
            elbowJoint.setPosition(60);
        }
    }

    @Override
    public void stop() {

    }


}
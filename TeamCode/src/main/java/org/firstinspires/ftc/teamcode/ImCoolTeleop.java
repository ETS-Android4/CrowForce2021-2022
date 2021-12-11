package org.firstinspires.ftc.teamcode;

import com.SCHSRobotics.HAL9001.system.robot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.robot.MainRobot;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ImCoolTeleop", group = "cool group")
public class ImCoolTeleop extends BaseTeleop {

    public @MainRobot MyRobot robot;

    @Override
    protected void onInit() {
        //robot.mDrive.reverseMotor("f_l_m");
        //robot.mDrive.reverseMotor("f_r_m");
        //robot.mDrive.reverseMotor("b_l_m");
        //robot.mDrive.reverseMotor("b_r_m");
        robot.spinner.spinMotor = hardwareMap.dcMotor.get("spin_motor");

    }

    @Override
    protected void onInitLoop() {

    }

    @Override
    protected void onStart() {
        
    }

    @Override
    protected void onUpdate() {
       
    }

    @Override
    protected void onStop() {

    }
}

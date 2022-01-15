package org.firstinspires.ftc.teamcode.Subsystem;

import android.app.backup.BackupAgent;

import com.SCHSRobotics.HAL9001.system.config.ConfigData;
import com.SCHSRobotics.HAL9001.system.config.ConfigParam;
import com.SCHSRobotics.HAL9001.system.config.TeleopConfig;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.SubSystem;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.SCHSRobotics.HAL9001.util.control.CustomizableGamepad;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Baguette;
import org.jetbrains.annotations.NotNull;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class DuckSpinner extends SubSystem {
    public CustomizableGamepad gamepad;
    //public Baguette robot;

    public static DcMotor spinMotor;
    public static boolean isSpinMotorButtonHeld;

    public static final String SPIN_MOTOR_BUTTON = "SPIN_MOTOR_BUTTON_Yeehaw";

    //public ConfigData data;
    //private int framesToSkip = 3;
    public DuckSpinner(@NotNull Baguette _robot, String _SPIN_MOTOR_CONFIG) {
        super(_robot);

        robot = _robot;
        spinMotor = _robot.hardwareMap.dcMotor.get(_SPIN_MOTOR_CONFIG);

        gamepad = new CustomizableGamepad(_robot);



        usesConfig = true;
    }

    public void spinSpinMotorTime(double _power, int time) {
        spinMotor.setPower(_power);
        waitTime(time);
        spinMotor.setPower(0);
    }

    public void spinSpinMotorTime() {
        spinMotor.setPower(0.5);
        waitTime(1000);
        spinMotor.setPower(0);
    }




    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        if (usesConfig && !robot.isAutonomous()) {
            //data = robot.pullNonGamepad(this);
            gamepad = robot.pullControls(this);


            //isSpinMotorButtonHeld = data.getData(SPIN_MOTOR_BUTTON, Boolean.class);
        }
    }

    @Override
    public void handle() {

        isSpinMotorButtonHeld = gamepad.getInput(SPIN_MOTOR_BUTTON);

        //isSpinMotorButtonHeld = gamepad1.a;
        spinMotor.setPower(0);
        if (isSpinMotorButtonHeld) spinMotor.setPower(.25);
    }

    @Override
    public void stop() {

    }

    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[] {
                new ConfigParam(SPIN_MOTOR_BUTTON, Button.BooleanInputs.a)
        };
    }
}
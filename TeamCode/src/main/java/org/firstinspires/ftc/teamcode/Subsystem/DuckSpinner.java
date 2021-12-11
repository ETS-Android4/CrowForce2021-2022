package org.firstinspires.ftc.teamcode.Subsystem;

import com.SCHSRobotics.HAL9001.system.config.ConfigData;
import com.SCHSRobotics.HAL9001.system.config.ConfigParam;
import com.SCHSRobotics.HAL9001.system.config.TeleopConfig;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.SubSystem;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.SCHSRobotics.HAL9001.util.control.CustomizableGamepad;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MyRobot;
import org.jetbrains.annotations.NotNull;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class DuckSpinner extends SubSystem {
    private CustomizableGamepad gamepad;
    public MyRobot robot;

    public DcMotor spinMotor;
    public static boolean isSpinMotorButtonHeld;

    public static final String SPIN_MOTOR_BUTTON = "SPIN_MOTOR_BUTTON";

    private ConfigData data;

    public DuckSpinner(@NotNull MyRobot _robot, String _SPIN_MOTOR_CONFIG) {
        super(_robot);

        gamepad = new CustomizableGamepad(_robot);

//        spinMotor = hardwareMap.dcMotor.get("spin_motor");

        robot = _robot;

        usesConfig = true;
    }

    public void spinSpinMotor(double _power) { spinMotor.setPower(_power); }

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        if (usesConfig) {
            gamepad = robot.pullControls(this);
            data = robot.pullNonGamepad(this);

            isSpinMotorButtonHeld = data.getData(SPIN_MOTOR_BUTTON, Boolean.class);
        }
    }

    @Override
    public void handle() {
        isSpinMotorButtonHeld = data.getData(SPIN_MOTOR_BUTTON, Boolean.class);
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

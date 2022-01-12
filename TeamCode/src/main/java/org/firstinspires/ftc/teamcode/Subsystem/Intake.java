package org.firstinspires.ftc.teamcode.Subsystem;

import com.SCHSRobotics.HAL9001.system.config.ConfigParam;
import com.SCHSRobotics.HAL9001.system.config.TeleopConfig;
import com.SCHSRobotics.HAL9001.system.robot.MainRobot;
import com.SCHSRobotics.HAL9001.system.robot.SubSystem;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.SCHSRobotics.HAL9001.util.control.CustomizableGamepad;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Baguette;

public class Intake extends SubSystem {
    public CustomizableGamepad gamepad;
    private Baguette robot;


    public static DcMotor slidesMotor;
    public static DcMotor intakeMotor;
    public static CRServo dropperServo;
    public static final String SLIDES_MOTOR_UP_BUTTON = "SLIDES_MOTOR_UP_BUTTON";
    public static final String SLIDES_MOTOR_DOWN_BUTTON = "SLIDES_MOTOR_DOWN_BUTTON";
    public static final String INTAKE_MOTOR_IN_BUTTON = "INTAKE_MOTOR_IN_BUTTON";
    public static final String INTAKE_MOTOR_OUT_BUTTON = "INTAKE_MOTOR_OUT_BUTTON";
    public static final String DROPPER_SERVO_BUTTON = "DROPPER_SERVO_BUTTON";

    boolean flag = true;



    public Intake(Baguette _robot, String _SLIDES_MOTOR_CONFIG, String _INTAKE_MOTOR_CONFIG, String _DROPPER_SERVO_CONFIG){
        super(_robot);

        slidesMotor = _robot.hardwareMap.dcMotor.get(_SLIDES_MOTOR_CONFIG);
        intakeMotor = _robot.hardwareMap.dcMotor.get(_INTAKE_MOTOR_CONFIG);
        dropperServo = _robot.hardwareMap.crservo.get(_DROPPER_SERVO_CONFIG);

        robot = _robot;

        gamepad = new CustomizableGamepad(_robot);



        usesConfig = true;
    }

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
        }
    }

    @Override
    public void handle() {
        if (gamepad.getInput(INTAKE_MOTOR_IN_BUTTON)){
            intakeMotor.setPower(0.5);
        }
        else if (gamepad.getInput(INTAKE_MOTOR_OUT_BUTTON)){
            intakeMotor.setPower(-0.5);
        }
        else {
            intakeMotor.setPower(0);
        }

        if (gamepad.getInput(SLIDES_MOTOR_UP_BUTTON)){
            slidesMotor.setPower(-.6);
        }
        else if (gamepad.getInput(SLIDES_MOTOR_DOWN_BUTTON)){
            slidesMotor.setPower(.6);
        }
        else {
            slidesMotor.setPower(0);
        }

        if (gamepad.getInput(DROPPER_SERVO_BUTTON)) {
            dropperServo.setPower(-.2);
            waitTime(750);
            dropperServo.setPower(-1);
        }
    }

    @Override
    public void stop() {

    }

    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[] {
                new ConfigParam(SLIDES_MOTOR_UP_BUTTON, Button.BooleanInputs.dpad_up),
                new ConfigParam(SLIDES_MOTOR_DOWN_BUTTON, Button.BooleanInputs.dpad_down),
                new ConfigParam(INTAKE_MOTOR_IN_BUTTON, Button.BooleanInputs.bool_right_trigger),
                new ConfigParam(INTAKE_MOTOR_OUT_BUTTON, Button.BooleanInputs.bool_left_trigger),
                new ConfigParam(DROPPER_SERVO_BUTTON , Button.BooleanInputs.x)
        };
    }
}

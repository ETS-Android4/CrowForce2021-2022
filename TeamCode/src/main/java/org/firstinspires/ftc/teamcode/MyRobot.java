package org.firstinspires.ftc.teamcode;

import com.SCHSRobotics.HAL9001.system.robot.ExternalCamera;
import com.SCHSRobotics.HAL9001.system.robot.InternalCamera;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class MyRobot extends Robot {
    @InternalCamera(resWidth = 320, resHeight = 240, usesViewport = true)
    @ExternalCamera(resWidth = 320, resHeight = 240, configName = "webcam", usesViewport = false)
    public OpenCvCamera camera = getCamera("webcam");


    public MyRobot(OpMode opMode) {
        super(opMode);
    }
}

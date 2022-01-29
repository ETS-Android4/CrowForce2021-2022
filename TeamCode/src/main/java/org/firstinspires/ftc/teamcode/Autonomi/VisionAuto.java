package org.firstinspires.ftc.teamcode.Autonomi;

import com.SCHSRobotics.HAL9001.system.robot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.system.robot.MainRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Baguette;
import org.firstinspires.ftc.teamcode.VisionStuff.Camera;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Vision", group = "Example Programs")
public class VisionAuto extends BaseAutonomous {
    public @MainRobot
    Baguette robot;
    //WeirdSpinnerHalWorkaround weird = new WeirdSpinnerHalWorkaround();

    @Override
    public void main() {
        OpenCvWebcam webcam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Cam"), cameraMonitorViewId);

        Camera pipeline;

        pipeline = new Camera();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1080, 720, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        waitTime(300000);
    }
}

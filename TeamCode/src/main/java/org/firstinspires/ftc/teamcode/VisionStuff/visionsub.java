package org.firstinspires.ftc.teamcode.VisionStuff;

import com.SCHSRobotics.HAL9001.system.robot.Camera;
import com.SCHSRobotics.HAL9001.system.robot.HALPipeline;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.VisionSubSystem;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class visionsub extends VisionSubSystem {
    private Pipeline myPipeline = new Pipeline();

    public visionsub(Robot robot) {
        super(robot);
    }
    /**
     * Gets all the HAL pipelines contained within this subsystem.
     *
     * @return All the HAL pipelines contained within this subsystem.
     */
    @Override
    protected HALPipeline[] getPipelines() {
        return new HALPipeline[] {new Pipeline()};
    }

    /**
     * A method containing the code that the subsystem runs when being initialized.
     */
    @Override
    public void init() {

    }

    /**
     * A method that contains code that runs in a loop on init.
     */
    @Override
    public void init_loop() {

    }

    /**
     * A method containing the code that the subsystem runs when being start.
     */
    @Override
    public void start() {

    }

    /**
     * A method containing the code that the subsystem runs every loop in a teleop program.
     */
    @Override
    public void handle() {

    }

    /**
     * A method containing the code that the subsystem runs when the program is stopped.
     */
    @Override
    public void stop() {

    }

    @Camera(id="webcam")
    public static class Pipeline extends HALPipeline {
        @Override
        public Mat processFrame(Mat input) {
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols() / 4,
                            input.rows() / 4),
                    new Point(
                            input.cols() * (3f / 4f),
                            input.rows() * (3f / 4f)),
                    new Scalar(0, 255, 0), 4);
            return input;
        }

        @Override
        public boolean useViewport() {
            return true;
        }
    }
}

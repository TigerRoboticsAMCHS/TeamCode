package org.firstinspires.ftc.teamcode.PowerPlay;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
@Disabled
public class AutoTest extends LinearOpMode {

    private DcMotor leftMotor, rightMotor;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.045;
    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    boolean found = false;
    int turn = 0;

    double wheelCircumference = 3.14159 * 0.09; // meters

    //pid stuff
    double integralSum = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;
    ElapsedTime timer  = new ElapsedTime();
    private double lastError = 0;
    private BHI260IMU imu;
    double referenceAngle = Math.toRadians(90);
    boolean doneTurning = false;
    public int idFound = -1;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BHI260IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(parameters);
        imu.resetYaw();

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.setMsTransmissionInterval(50);
        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(10000);

        while (opModeIsActive()) {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if (detections != null) {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
                telemetry.addData("rotation", Math.toDegrees(getYaw()));
                telemetry.addData("target", Math.toDegrees(referenceAngle));
                telemetry.addData("turn", turn);

                // If we don't see any tags
                if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for (AprilTagDetection detection : detections) {
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                        idFound = detection.id;
                    }
                    if(!found) {
                        driveDistance(0.3, 0.6096);
                        switch (idFound) {
                            case 0:
                                turn = 1;
                                break;
                            case 1:
                                turn = 0;
                                break;
                            case 2:
                                turn = -1;
                                break;
                        }
                        referenceAngle += getYaw();
                        found = true;
                    }
                    found = true;
                }
                telemetry.update();
            }
            if(found && !doneTurning) {
//                double power = PIDControl(referenceAngle, getYaw());
//                telemetry.addData("motor power", power);
//                leftMotor.setPower(0.2 * turn);
//                rightMotor.setPower(-0.2 * turn);
//                telemetry.addData("power", leftMotor.getPower());
//                telemetry.addData("power", rightMotor.getPower());
//                if(Math.toDegrees(Math.abs(referenceAngle - getYaw())) < 2)
//                {
//                    turn = 0;
//                    leftMotor.setPower(0);
//                    rightMotor.setPower(0);
//                    doneTurning = true;
//                }
                rotate90(turn);
                doneTurning = true;
            }
            if(found && doneTurning) {
                driveDistance(0.3, 0.6096);
                found = !found;
            }
        }
    }
    public void driveDistance(double power, double distance) {
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double rotationsNeeded = distance/wheelCircumference;
        int targetValue = (int) (rotationsNeeded * 537.6);
        rightMotor.setTargetPosition(targetValue);
        leftMotor.setTargetPosition(targetValue);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setPower(power);
        leftMotor.setPower(power);
        while(leftMotor.isBusy() || rightMotor.isBusy()) {
            telemetry.addData("Right encoder value", rightMotor.getCurrentPosition());
            telemetry.addData("left encoder value", leftMotor.getCurrentPosition());
        }
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }
    public void rotate90(int turn) {
        switch (idFound) {
            case 0:
                turn = 1;
                break;
            case 1:
                turn = 0;
                break;
            case 2:
                turn = -1;
                break;
        }
        while(Math.toDegrees(Math.abs(referenceAngle - getYaw())) > 2)
        {
            leftMotor.setPower(0.2 * turn);
            rightMotor.setPower(-0.2 * turn);
            telemetry.addData("power", leftMotor.getPower());
            telemetry.addData("power", rightMotor.getPower());
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        doneTurning = true;
    }
    public double PIDControl(double reference, double state) {
        double error = angleWrap(reference - state);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
    public double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    public double angleWrap(double radians) {
        // allows us to turn the shortest distance to what we want
        while(radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
}

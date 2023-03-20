package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "main")
public class FieldCentricDrive extends LinearOpMode {
    private DcMotor BackRight, BackLeft, FrontRight, FrontLeft;
    private IMU imu;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        imu.resetYaw();
        double init = getYaw();
        waitForStart();
        while(opModeIsActive()) {
//            double FrontLeftVal =  gamepad1.left_stick_y - (gamepad1.left_stick_x)  + -gamepad1.right_stick_x;
//            double FrontRightVal =  gamepad1.left_stick_y  + (gamepad1.left_stick_x) - -gamepad1.right_stick_x;
//            double BackLeftVal = gamepad1.left_stick_y  + (gamepad1.left_stick_x)  + -gamepad1.right_stick_x;
//            double BackRightVal = gamepad1.left_stick_y - (gamepad1.left_stick_x) - -gamepad1.right_stick_x;
            double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + getYaw() - init;
            double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2.0) + Math.pow(gamepad1.left_stick_y, 2.0));
            double FrontLeftVal = Math.sin(angle - 0.25 * Math.PI) * magnitude - 0.66 * gamepad1.right_stick_x;
            double FrontRightVal = Math.sin(angle + 0.25 * Math.PI) * magnitude + 0.66 * gamepad1.right_stick_x;
            double BackLeftVal = Math.sin(angle + 0.25 * Math.PI) * magnitude - 0.66 * gamepad1.right_stick_x;
            double BackRightVal = Math.sin(angle - 0.25 * Math.PI) * magnitude + 0.66 * gamepad1.right_stick_x;
            // Move range to between 0 and +1, if not already
//            double[] wheelPowers = {FrontRightVal, FrontLeftVal, BackLeftVal, BackRightVal};
//            Arrays.sort(wheelPowers);
//            if (wheelPowers[3] > 1) {
//                FrontLeftVal /= wheelPowers[3];
//                FrontRightVal /= wheelPowers[3];
//                BackLeftVal /= wheelPowers[3];
//                BackRightVal /= wheelPowers[3];
//            }
            FrontLeftVal /= 1.66;
            FrontRightVal /= 1.66;
            BackLeftVal /= 1.66;
            BackRightVal /= 1.66;

            drive.setMotorPowers(FrontLeftVal, BackLeftVal, BackRightVal, FrontRightVal);

            telemetry.addData("FL", FrontLeftVal);
            telemetry.addData("FR", FrontRightVal);
            telemetry.addData("BL", BackLeftVal);
            telemetry.addData("BR", BackRightVal);
            telemetry.addData("yaw", Math.toDegrees(getYaw()));
            telemetry.update();
        }
    }
    public double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;

@TeleOp(group = "main")
public class TestDrive extends LinearOpMode {
    private DcMotor BackRight, BackLeft, FrontRight, FrontLeft;

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
        waitForStart();
        while(opModeIsActive()) {
            double FrontLeftVal =  gamepad1.left_stick_y - (gamepad1.left_stick_x)  + -gamepad1.right_stick_x;
            double FrontRightVal =  gamepad1.left_stick_y  + (gamepad1.left_stick_x) - -gamepad1.right_stick_x;
            double BackLeftVal = gamepad1.left_stick_y  + (gamepad1.left_stick_x)  + -gamepad1.right_stick_x;
            double BackRightVal = gamepad1.left_stick_y - (gamepad1.left_stick_x) - -gamepad1.right_stick_x;

            //Move range to between 0 and +1, if not already
            double[] wheelPowers = {FrontRightVal, FrontLeftVal, BackLeftVal, BackRightVal};
            Arrays.sort(wheelPowers);
            if (wheelPowers[3] > 1) {
                FrontLeftVal /= wheelPowers[3];
                FrontRightVal /= wheelPowers[3];
                BackLeftVal /= wheelPowers[3];
                BackRightVal /= wheelPowers[3];
            }
            FrontLeftVal *= 2.0/3.0;
            FrontRightVal *= 2.0/3.0;
            BackLeftVal *= 2.0/3.0;
            BackRightVal *= 2.0/3.0;

            FrontLeft.setPower(FrontLeftVal);
            FrontRight.setPower(FrontRightVal);
            BackLeft.setPower(BackLeftVal);
            BackRight.setPower(BackRightVal);

            telemetry.addData("FL", FrontLeftVal);
            telemetry.addData("FR", FrontRightVal);
            telemetry.addData("BL", BackLeftVal);
            telemetry.addData("BR", BackRightVal);
            telemetry.update();
        }
    }
}

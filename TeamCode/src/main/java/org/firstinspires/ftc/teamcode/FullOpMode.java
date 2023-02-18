package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FullOpMode extends LinearOpMode {
    private DcMotor leftLift, rightLift;
    private DcMotor leftMotor, rightMotor;
    private Servo claw;
    private Servo claw2;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(0);
        claw2 = hardwareMap.get(Servo.class, "claw2");
        claw2.setPosition(0);

        double liftPower = 0;
        double rightPower = 0, leftPower = 0;

        telemetry.addData("claw position", claw.getPosition());
        telemetry.addData("Status", "Sus");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            //movement
            if (Math.abs(gamepad1.left_stick_x) > Math.abs(gamepad1.left_stick_y)) {
                // minimize turn, too fast
                rightPower = gamepad1.left_stick_x / 3;
                leftPower = gamepad1.left_stick_x / 3;
            } else if (gamepad1.left_stick_x == gamepad1.left_stick_y) {
                //nothing
                rightPower = 0;
                leftPower = 0;
            } else {
                // forwards, this is forwards because motors are inverted
                rightPower = gamepad1.left_stick_y;
                leftPower = -gamepad1.left_stick_y;
            }
            //set power
            rightMotor.setPower(rightPower);
            leftMotor.setPower(leftPower);
            telemetry.addData("Right Drive", rightMotor.getPower());
            telemetry.addData("Left Drive", leftMotor.getPower());

            //grab
            if(Math.abs(gamepad2.right_stick_x) > Math.abs(gamepad2.right_stick_y)) {
                if(gamepad2.right_stick_x > 0) {
                    claw.setPosition(claw.getPosition() - 0.001);
                }
                if(gamepad2.right_stick_x < 0) {
                    claw.setPosition(claw.getPosition() + 0.001);
                }
            }
            else {
                if(gamepad2.right_stick_y < 0) {
                    claw2.setPosition(claw2.getPosition() + 0.001);
                }
                if(gamepad2.right_stick_y > 0) {
                    claw2.setPosition(claw2.getPosition() - 0.001);
                }
            }
            telemetry.addData("Claw Position", claw.getPosition());
            telemetry.addData("Rotating thingy Position", claw2.getPosition());

            //lift
            liftPower = -gamepad2.left_stick_y;
            //set power
            rightLift.setPower(liftPower);
            leftLift.setPower(-liftPower);
            telemetry.addData("Right Lift", rightLift.getPower());
            telemetry.addData("Left Lift", leftLift.getPower());

            telemetry.addData("sUssy", "sus");
            telemetry.update();
        }
    }
}

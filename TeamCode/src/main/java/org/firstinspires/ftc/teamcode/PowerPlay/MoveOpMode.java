package org.firstinspires.ftc.teamcode.PowerPlay;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class MoveOpMode extends LinearOpMode {

    private DcMotor leftMotor, rightMotor;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        telemetry.addData("Status", "Sus");
        telemetry.update();

        waitForStart();
        double rightPower = 0, leftPower = 0;
        while(opModeIsActive())
        {
            if (Math.abs(gamepad1.left_stick_x) > Math.abs(gamepad1.left_stick_y)){
                rightPower = gamepad1.left_stick_x;
                leftPower = gamepad1.left_stick_x;
            }else if(gamepad1.left_stick_x == gamepad1.left_stick_y){
                rightPower = 0;
                leftPower = 0;
            }else{
                rightPower = gamepad1.left_stick_y;
                leftPower = -gamepad1.left_stick_y;
            }//xd your mom lmfao lmoao la
            rightMotor.setPower(rightPower);
            leftMotor.setPower(leftPower);
            telemetry.addData("Right", rightMotor.getPower());
            telemetry.addData("Left", leftMotor.getPower());
            telemetry.addData("Status", "sus");
            telemetry.update();
        }
    }
}

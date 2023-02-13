package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class LiftOPMode extends LinearOpMode {

    private DcMotor leftLift, rightLift;

    @Override
    public void runOpMode() {
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        telemetry.addData("Status", "Sus");
        telemetry.update();

        waitForStart();
        double liftPower = 0;
        while(opModeIsActive())
        {
            liftPower = -this.gamepad1.right_stick_y;
            rightLift.setPower(liftPower);
            leftLift.setPower(-liftPower);
            telemetry.addData("Right", rightLift.getPower());
            telemetry.addData("Left", leftLift.getPower());
            telemetry.addData("Status", "sus");
            telemetry.update();
        }
    }
}

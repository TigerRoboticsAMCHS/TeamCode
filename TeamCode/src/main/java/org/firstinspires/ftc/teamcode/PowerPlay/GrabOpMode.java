package org.firstinspires.ftc.teamcode.PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class GrabOpMode extends LinearOpMode {
    private Servo claw;
    @Override
    public void runOpMode() {
        claw = hardwareMap.get(Servo.class, "claw");
        telemetry.addData("Status", "Sus");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a) {
                claw.setPosition(0);
            }
            else if(gamepad1.b) {
                claw.setPosition(0.4);
            }
            telemetry.addData("Servo Position", claw.getPosition());
            telemetry.addData("sUssy", "sus");
            telemetry.update();
        }
    }
}

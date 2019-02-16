package org.firstinspires.ftc.teamcode.Samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
//@Disabled
public class TestProgram extends LinearOpMode {
    DcMotor hangingMotor = null;

    @Override
    public void runOpMode() {
        hangingMotor = hardwareMap.get(DcMotor.class, "HangingArm");
        waitForStart();
        while (opModeIsActive()) {
            hangingMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        }
    }
}
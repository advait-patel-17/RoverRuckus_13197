package org.firstinspires.ftc.teamcode.RoverRuckusOnSeason.Samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
//@Disabled
public class TestProgram extends LinearOpMode {
    Servo Test = null;
    Servo Test1 = null;

    @Override
    public void runOpMode() {
        Test1 = hardwareMap.get(Servo.class, "Dumper");
        Test = hardwareMap.get(Servo.class, "IntakeFlip1");
        waitForStart();
        while (opModeIsActive()) {
            Test.setPosition(gamepad1.right_trigger);
            Test1.setPosition(gamepad1.left_trigger);
            telemetry.addData("intake position", Test.getPosition());
            telemetry.addData("dumper position", Test1.getPosition());
            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode.Samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
//@Disabled
public class Test extends LinearOpMode {
    CRServo TestServo1;
    CRServo TestServo2;
    Servo TestServo3;

    @Override
    public void runOpMode(){
        TestServo1 = hardwareMap.get(CRServo.class, "IntakeFlip1");
        TestServo2 = hardwareMap.get(CRServo.class, "IntakeFlip2");
        TestServo3 = hardwareMap.get(Servo.class, "Dumper");
        TestServo2.setDirection(CRServo.Direction.REVERSE);
        waitForStart();
        TestServo1.setPower(0.8);
        TestServo2.setPower(0.8);
        TestServo3.setPosition(0.5);
        sleep(3000);
        TestServo1.setPower(0);
        TestServo2.setPower(0);
        telemetry.addData("1 Power", TestServo1.getPower());
        telemetry.addData("2 Power", TestServo2.getPower());
        telemetry.addData("3 Position", TestServo3.getPosition());
        telemetry.update();
    }
}
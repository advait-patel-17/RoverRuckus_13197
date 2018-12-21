
package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
//@Disabled
public class TestProgram extends LinearOpMode {
    Servo teamMarker;
    public void runOpMode(){
        teamMarker = hardwareMap.get(Servo.class, "TeamMarker");
        waitForStart();
        while(opModeIsActive()){
            teamMarker.setPosition(gamepad1.right_trigger);
            telemetry.addData("Position", teamMarker.getPosition());
            telemetry.update();
        }
    }
}

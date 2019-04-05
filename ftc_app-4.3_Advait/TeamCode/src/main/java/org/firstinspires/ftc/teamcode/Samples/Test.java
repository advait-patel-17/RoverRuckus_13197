package org.firstinspires.ftc.teamcode.Samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class Test extends LinearOpMode {
    DcMotor Sample = null;

    @Override
    public void runOpMode(){
        Sample = hardwareMap.get(DcMotor.class, "FrontRight");
        waitForStart();
        Sample.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Sample.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Sample.setTargetPosition(1000);
        Sample.setPower(1);
        while(Sample.isBusy()&&opModeIsActive()){
            telemetry.addData("Ticks", Sample.getCurrentPosition());
            telemetry.update();
        }
        Sample.setPower(0);
        sleep(10000);

    }
}
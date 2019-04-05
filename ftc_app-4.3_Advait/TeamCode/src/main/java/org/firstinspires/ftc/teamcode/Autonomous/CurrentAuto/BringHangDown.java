package org.firstinspires.ftc.teamcode.Autonomous.CurrentAuto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class BringHangDown extends LinearOpMode {
    private DcMotor Lift = null;

    @Override
    public void runOpMode () {
        Lift = hardwareMap.get(DcMotor.class, "HangingArm");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        Lift.setPower(-1);
        sleep(2000);
        Lift.setPower(0);
    }
}

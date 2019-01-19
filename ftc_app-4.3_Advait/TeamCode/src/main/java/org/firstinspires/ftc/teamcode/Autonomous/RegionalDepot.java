package org.firstinspires.ftc.teamcode.Autonomous;

import android.database.sqlite.SQLiteException;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Vision;

@Autonomous
@Disabled
public class RegionalDepot extends LinearOpMode{


    Robot robot = new Robot();
    Vision vision = new Vision();



    @Override
    public void runOpMode(){

        robot.initRobot(hardwareMap);
        vision.initVision();

        waitForStart();

        robot.hangingMotor.setPower(-1);
        sleep(3900);
        robot.hangingMotor.setPower(0);

        robot.strafe(0.5, 450, this);
        sleep(300);

        robot.moveWithEncoders(-0.3, 400, this);

        if (opModeIsActive()) {
            // Activate Tensor Flow Object Detection.
            if (vision.tfod != null) {
                vision.tfod.activate();
            }
        }

        robot.hangingMotor.setPower(1);

        boolean isGold = vision.checkForGold(this);
        byte mineralPosition;

        if (isGold){
            if (vision.degreesToGold(this)<0){
                mineralPosition = 1;
            }
            else {
                mineralPosition = 2;
            }
        }

        else {
            mineralPosition = 3;
        }
        robot.hangingMotor.setPower(0);
        robot.moveWithEncoders(0.5, 600, this);
        robot.intakeExtender.setPower(1);
        sleep(1200);
        robot.intakeExtender.setPower(0);
        robot.intakeFlip1.setPower(-1);
        robot.intakeFlip2.setPower(-1);
        sleep(1000);
        robot.intakeFlip1.setPower(0);
        robot.intakeFlip2.setPower(0);
        robot.intake.setPower(-1);
        sleep(1200);
        robot.intake.setPower(0);
        robot.intakeFlip1.setPower(1);
        robot.intakeFlip2.setPower(1);
        robot.intakeExtender.setPower(-1);
        sleep(1000);
        robot.intakeFlip1.setPower(0);
        robot.intakeFlip2.setPower(0);
        sleep(200);
        robot.intakeExtender.setPower(0);
        sleep(300);
        robot.moveWithEncoders(-0.3, 400, this);
        switch (mineralPosition){
            case 1:
                robot.strafe(0.5, 400, this);
                robot.intakeFlip1.setPower(-1);
                robot.intakeFlip2.setPower(-1);
                sleep(1000);
                robot.intakeFlip1.setPower(0);
                robot.intakeFlip2.setPower(0);
                sleep(300);
                robot.intake.setPower(1);
                robot.moveWithEncoders(0.5, 500, this);
                sleep(300);
                robot.intake.setPower(0);
                robot.intakeFlip1.setPower(-1);
                robot.intakeFlip2.setPower(-1);
                sleep(1000);
                robot.intakeFlip1.setPower(0);
                robot.intakeFlip2.setPower(0);
                sleep(300);
                robot.IMUTurn(90, 0.3, -0.3, 2, this);
                sleep(300);
                robot.moveWithEncoders(0.5, 1000, this);
        }
    }
}
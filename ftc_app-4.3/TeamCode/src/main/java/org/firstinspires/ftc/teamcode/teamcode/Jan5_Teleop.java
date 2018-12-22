package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Jan5_Teleop", group="Linear Opmode")
//@Disabled
public class Jan5_Teleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor hangingMotor = null;
    private DcMotor intake = null;
    private DcMotor intakeExtender = null;
    private DcMotor scoring = null;
    private Servo dumper = null;
    private CRServo intakeFlip1 = null;
    private CRServo intakeFlip2 = null;


    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
       initMotors();

       double shift = 1;



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.left_trigger-gamepad1.right_trigger;
            final double v1 = Range.clip(r * Math.cos(robotAngle) + rightX, -1, 1);
            final double v2 = Range.clip(r * Math.sin(robotAngle) - rightX, -1, 1);
            final double v3 = Range.clip(r * Math.sin(robotAngle) + rightX, -1, 1);
            final double v4 = Range.clip(r * Math.cos(robotAngle) - rightX, -1, 1);

            leftFrontDrive.setPower(v1/shift);
            rightFrontDrive.setPower(v2/shift);
            leftBackDrive.setPower(v3/shift);
            rightBackDrive.setPower(v4/shift);

            scoring.setPower(gamepad2.left_stick_y);
            intake.setPower(gamepad2.left_trigger-gamepad2.right_trigger);
            intakeFlip2.setPower(gamepad2.right_stick_y);
            intakeFlip1.setPower(gamepad2.right_stick_y);
            intakeExtender.setPower(gamepad1.right_stick_y);

            if (gamepad2.dpad_up){
                hangingMotor.setPower(1);
            }
            else if (gamepad2.dpad_down){
                hangingMotor.setPower(-1);
            }
            else {
                hangingMotor.setPower(0);
            }

            if (gamepad2.a){
                dumper.setPosition(0);
            }
            else if (gamepad2.b){
                dumper.setPosition(0.3);
            }


            if (gamepad1.right_bumper){
                shift = 2;
            }

            else {
                shift = 1;
            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("LeftFront Power", v1);
            telemetry.addData("RightFront Power", v2);
            telemetry.addData("LeftBack Power", v3);
            telemetry.addData("RightBack Power", v4);
            telemetry.addData("Shift", shift);
            telemetry.update();


        }
    }

    public void initMotors() {
        leftBackDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BackRight");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        hangingMotor = hardwareMap.get(DcMotor.class, "HangingArm");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        intakeExtender = hardwareMap.get(DcMotor.class, "IntakeExtend");
        scoring = hardwareMap.get(DcMotor.class, "Scoring");
        intakeFlip1 = hardwareMap.get(CRServo.class, "IntakeFlip1");
        intakeFlip2 = hardwareMap.get(CRServo.class, "IntakeFlip2");
        dumper = hardwareMap.get(Servo.class, "Dumper");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeFlip1.setDirection(CRServo.Direction.FORWARD);
        intakeFlip2.setDirection(CRServo.Direction.REVERSE);

        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scoring.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        intake.setPower(0);
        intakeExtender.setPower(0);
        hangingMotor.setPower(0);
        intakeFlip1.setPower(0);
        intakeFlip2.setPower(0);
        scoring.setPower(0);
    }

}

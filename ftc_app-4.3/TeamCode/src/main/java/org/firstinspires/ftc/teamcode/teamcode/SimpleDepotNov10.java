package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="SimpleDepotNov10", group="Linear Opmode")
@Disabled
public class SimpleDepotNov10 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor hangingMotor = null;
    private DcMotor intake = null;
    private DcMotor intakeFlipper = null;
    private DcMotor intakeExtender = null;
    private Servo samplingArm = null;
    private Servo teamMarker = null;

    //Color Sensor
    ColorSensor sensorColorRight;
    ColorSensor sensorColorLeft;

    // The IMU sensor object
    BNO055IMU imu1;

    // State used for updating telemetry
    Orientation angles;

    @Override
    public void runOpMode() {
        leftBackDrive  = hardwareMap.get(DcMotor.class, "BackLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BackRight");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        hangingMotor = hardwareMap.get(DcMotor.class, "HangingArm");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        intakeExtender = hardwareMap.get(DcMotor.class, "IntakeExtend");
        intakeFlipper = hardwareMap.get(DcMotor.class, "IntakeFlipper");
        samplingArm = hardwareMap.get(Servo.class, "SamplingArm");
        teamMarker = hardwareMap.get(Servo.class, "TeamMarker");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeFlipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        hangingMotor.setPower(0);
        intake.setPower(0);
        intakeFlipper.setPower(0);
        intakeExtender.setPower(0);

        sleep(500);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //wait for the driver to press start
        waitForStart();
        runtime.reset();

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*leftFrontDrive.setTargetPosition(1300);
        leftBackDrive.setTargetPosition(1300);
        rightFrontDrive.setTargetPosition(1300);
        rightBackDrive.setTargetPosition(1300);
*/
        leftBackDrive.setPower(0.3);
        leftFrontDrive.setPower(0.3);
        rightBackDrive.setPower(0.3);
        rightFrontDrive.setPower(0.3);

        sleep(2700);


        /*while (opModeIsActive() && ((leftFrontDrive.getCurrentPosition()+rightBackDrive.getCurrentPosition()+rightFrontDrive.getCurrentPosition()+leftBackDrive.getCurrentPosition())/4) < 1300){
            telemetry.addData("Left Front Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("Left Back Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("Right Front Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("Right Back Position", rightBackDrive.getCurrentPosition());
            telemetry.addData("Power", leftBackDrive.getPower());
            telemetry.addData("Target Position", leftBackDrive.getTargetPosition());
            telemetry.update();
        }
*/
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);

        sleep(500);

        double originalPosition = teamMarker.getPosition();
        sleep(200);
        teamMarker.setPosition(0.85);

        sleep(3000);


        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBackDrive.setPower(-0.3);
        leftFrontDrive.setPower(-0.3);
        rightBackDrive.setPower(-0.3);
        rightFrontDrive.setPower(-0.3);

        sleep(1000);

        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);

        sleep(500);


        teamMarker.setPosition(originalPosition);

        sleep(27000);

    }
}
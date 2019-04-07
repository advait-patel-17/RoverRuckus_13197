package org.firstinspires.ftc.teamcode.RoverRuckusOnSeason.Autonomous.PastAuto;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
@Disabled
public class SimpleDepotDec1 extends LinearOpMode{

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

    BNO055IMU imu1;
    Orientation angles;

    @Override
    public void runOpMode() {
        initMotors();
        initIMU();
        initServos();

        telemetry.addData("Status", "Initialized. Good to go!");
        telemetry.update();
        waitForStart();
        runtime.reset();

        hangingMotor.setPower(1);
        sleep(5000);
        hangingMotor.setPower(0);

        sleep(500);

        strafe(0.5, 500);
        sleep(500);

        moveWithEncoders(0.5, 500);
        sleep(500);

        strafe(-0.5, 600);
        sleep(500);

        leftBackDrive.setPower(-0.5);
        leftFrontDrive.setPower(-0.5);
        rightBackDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        sleep(1700);
        stopDrivetrain();
        sleep(500);

        moveWithEncoders(-0.5, 1500);
        sleep(500);

        teamMarker.setPosition(0);
        sleep(800);

        moveWithEncoders(0.5, 1000);
        sleep(500);
/*
        IMUTurn(-45, -0.5, 0.2, 3);
        sleep(500);

        moveWithEncoders(0.5, 500);
        strafe(0.3, 1000);
        sleep(500);

        moveWithEncoders(0.5, 3000);
        sleep(500);
        intake.setPower(-0.5);
        sleep(700);
        intake.setPower(0);
        sleep(27000);
  */  }

    public void initMotors(){
        leftBackDrive  = hardwareMap.get(DcMotor.class, "BackLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BackRight");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        hangingMotor = hardwareMap.get(DcMotor.class, "HangingArm");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        intakeExtender = hardwareMap.get(DcMotor.class, "IntakeExtend");
        intakeFlipper = hardwareMap.get(DcMotor.class, "IntakeFlipper");


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

        stopDrivetrain();
        hangingMotor.setPower(0);
        intake.setPower(0);
        intakeFlipper.setPower(0);
        intakeExtender.setPower(0);

        sleep(500);
    }


    public void initServos(){
        samplingArm = hardwareMap.get(Servo.class, "Sample");
        teamMarker = hardwareMap.get(Servo.class, "TeamMarker");

        samplingArm.setPosition(0);
    }

    public void moveWithEncoders(double motorPower, int sleepTime){
        useEncoders();

        leftFrontDrive.setPower(motorPower);
        leftBackDrive.setPower(motorPower);
        rightBackDrive.setPower(motorPower);
        rightFrontDrive.setPower(motorPower);

        sleep(sleepTime);
        stopDrivetrain();
    }


    public void useEncoders(){
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafe(double power, int sleepTime){
        useEncoders();

        leftBackDrive.setPower(-power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);

        sleep(sleepTime);
        stopDrivetrain();
    }


    public void strafeAngle(double speed, double radians, int sleepTime){
        useEncoders();

        double robotAngle = radians - (Math.PI/4);
        double[] actualSpeed;
        actualSpeed = new double[4];
        actualSpeed[0] = Math.cos(robotAngle)*speed;
        actualSpeed[1] = Math.sin(robotAngle)*speed;
        actualSpeed[2] = Math.sin(robotAngle)*speed;
        actualSpeed[3] = Math.cos(robotAngle)*speed;

        leftFrontDrive.setPower(actualSpeed[0]);
        rightFrontDrive.setPower(actualSpeed[1]);
        leftBackDrive.setPower(actualSpeed[2]);
        rightBackDrive.setPower(actualSpeed[3]);

        sleep(sleepTime);

        stopDrivetrain();
    }

    public void initIMU(){
        // Initialize the hardware variables. Note that the strings used here as parameters

        //Initialization initi = new Initialization();
        // initi.IMUinit();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu1 = hardwareMap.get(BNO055IMU.class, "imu 2");
        imu1.initialize(parameters);
        angles   = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }


    public void IMUTurn(double target, double rightPower, double leftPower, double threshold){
        double currentValue = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double realTarget = currentValue + target;
        useEncoders();

        leftFrontDrive.setPower(leftPower);
        leftBackDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        rightBackDrive.setPower(rightPower);

        while(Math.abs(realTarget-currentValue)>threshold && opModeIsActive()){
            currentValue = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            telemetry.addData("Current Value", currentValue);
            telemetry.addData("Target", target);
            telemetry.update();
        }

        stopDrivetrain();

    }

    public void stopDrivetrain(){
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
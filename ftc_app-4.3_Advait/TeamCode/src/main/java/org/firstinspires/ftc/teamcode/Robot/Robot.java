package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Disabled
public class Robot extends LinearOpMode{
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;

    public DcMotor hangingMotor = null;
    public DcMotor intake = null;
    public DcMotor intakeExtender = null;
    public DcMotor scoring = null;
    public Servo dumper = null;
    public CRServo intakeFlip1 = null;
    public CRServo intakeFlip2 = null;

    public HardwareMap hwMap = null;

    public BNO055IMU imu;
    public Orientation angles;

    @Override
    public void runOpMode() {

    }

    public void initRobot(HardwareMap ahwmap){
        hwMap = ahwmap;
        leftBackDrive = hwMap.get(DcMotor.class, "BackLeft");
        rightBackDrive = hwMap.get(DcMotor.class, "BackRight");
        leftFrontDrive = hwMap.get(DcMotor.class, "FrontLeft");
        rightFrontDrive = hwMap.get(DcMotor.class, "FrontRight");
        hangingMotor = hwMap.get(DcMotor.class, "HangingArm");
        intake = hwMap.get(DcMotor.class, "Intake");
        intakeExtender = hwMap.get(DcMotor.class, "IntakeExtend");
        scoring = hwMap.get(DcMotor.class, "Scoring");
        intakeFlip1 = hwMap.get(CRServo.class, "IntakeFlip1");
        intakeFlip2 = hwMap.get(CRServo.class, "IntakeFlip2");
        dumper = hwMap.get(Servo.class, "Dumper");



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
        imu = hwMap.get(BNO055IMU.class, "imu 2");
        imu.initialize(parameters);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }

    public void moveWithEncoders(double motorPower, int sleepTime, LinearOpMode om){
        useEncoders();

        leftFrontDrive.setPower(motorPower);
        leftBackDrive.setPower(motorPower);
        rightBackDrive.setPower(motorPower);
        rightFrontDrive.setPower(motorPower);

        om.sleep(sleepTime);

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
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

    public void strafe(double power, int sleepTime, LinearOpMode om){
        useEncoders();

        leftBackDrive.setPower(-power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);

        om.sleep(sleepTime);

        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }


    public void strafeAngle(double speed, double radians, int sleepTime, LinearOpMode om){
        useEncoders();

        double robotAngle = radians + (Math.PI/4);
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

        om.sleep(sleepTime);

        stopDrivetrain();
    }

    public void IMUTurn(double target, double rightPower, double leftPower, double threshold, LinearOpMode om){
        double currentValue = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        target = Math.toRadians(target);
        double realTarget = currentValue + target;
        double kp = 1.3;
        double error;
        double leftOutput;
        double rightOutput;
        threshold = Math.toRadians(threshold);
        useEncoders();


        while(Math.abs(realTarget-currentValue)>threshold && opModeIsActive()){
            error = realTarget-currentValue;
            leftOutput = Range.clip(error*kp*leftPower, -1, 1);
            rightOutput = Range.clip(error*kp*rightPower, -1, 1);

            leftFrontDrive.setPower(leftOutput);
            leftBackDrive.setPower(leftOutput);
            rightFrontDrive.setPower(rightOutput);
            rightBackDrive.setPower(rightOutput);

            currentValue = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
            telemetry.addData("Current Value", currentValue);
            telemetry.addData("Target", target);
            telemetry.addData("Left Power", leftBackDrive.getPower());
            telemetry.addData("Right Power", rightBackDrive.getPower());
            telemetry.update();
            om.sleep(50);
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

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

@Autonomous(name="Nov10_BlueDepot", group="Linear Opmode")
@Disabled
public class Nov10_BlueDepot extends LinearOpMode {
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
    public void runOpMode(){
        //Initialize Motors
        initMotors();
        //Initialize IMU
        initIMU();
        //Initialize Servos
        initServos();
        //Initialize Color Sensor
        initColorSensors();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //wait for the driver to press start
        waitForStart();
        runtime.reset();

        //Go forward to the Sampling minerals
        setMotorPosition(500, 0.2);
        sleep(400);

        IMUTurning(-0.3, 0.3, 90);

        //Put down the sampling arm
        samplingArm.setPosition(0.5);
        sleep(500);

        /*//set the color variables. Left signifies the left color sensor and right signifies the other one
        double redRight = sensorColorRight.red();
        double greenRight = sensorColorRight.green();
        double blueRight = sensorColorRight.blue();

        double redLeft = sensorColorLeft.red();
        double greenLeft = sensorColorLeft.green();
        double blueLeft = sensorColorLeft.blue();

        //if mineral is detected...
        if (redRight>150 && greenRight>100 && blueRight<100){
            //tell drivers mineral is found
            telemetry.addData("Status", "Detected Mineral");
            telemetry.update();

            moveWithEncoders(0.2, 500);
            sleep(500);

            samplingArm.setPosition(0);
            sleep(500);

            moveWithEncoders(0.2, 700);
            sleep(500);
        }
        else {
            telemetry.addData("Status:", "Mineral not found");
            telemetry.update();

            samplingArm.setPosition(0);

            moveWithEncoders(-0.2, 700);
            sleep(500);

            samplingArm.setPosition(0.5);
            sleep(500);

            if (redLeft> 150 && greenLeft>100 && blueLeft<100){
                telemetry.addData("Status", "Mineral Detected");
                telemetry.update();

                moveWithEncoders(0.2, 700);
                sleep(500);

                samplingArm.setPosition(0);
                sleep(500);

                moveWithEncoders(0.2, 700);
                sleep(1000);


            }
            else {
                telemetry.addData("Status", "Mineral not found");
                telemetry.update();

                samplingArm.setPosition(0);
                sleep(500);

                moveWithEncoders(0.2, 800);
                sleep(500);

                samplingArm.setPosition(0.5);
                sleep(500);

                moveWithEncoders(0.2,500);
                sleep(500);

                samplingArm.setPosition(0);
                sleep(500);
            }
        }
        setMotorPosition(600, 0.2);
        sleep(500);

        IMUTurning(-0.2, 0.1, 45);
        moveWithEncoders(0.2, 700);
*/


    }


    public void initServos(){
        samplingArm = hardwareMap.get(Servo.class, "SamplingArm");
        teamMarker = hardwareMap.get(Servo.class, "TeamMarker");
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

    public void initColorSensors(){
        // get a reference to the color sensor.
        sensorColorRight = hardwareMap.get(ColorSensor.class, "RightSensor");

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attenuate the measured values.
        final double SCALE_FACTOR = 255;

        //get a reference to the color sensor.
        sensorColorLeft = hardwareMap.get(ColorSensor.class, "LeftSensor");
    }

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

        leftBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        hangingMotor.setPower(0);
        intake.setPower(0);
        intakeFlipper.setPower(0);
        intakeExtender.setPower(0);

        sleep(500);


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

    public void moveWithEncoders(double motorPower, int sleepTime){
        useEncoders();

        leftFrontDrive.setPower(motorPower);
        leftBackDrive.setPower(motorPower);
        rightBackDrive.setPower(motorPower);
        rightFrontDrive.setPower(motorPower);

        sleep(sleepTime);

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);

    }
    public void setMotorPosition(int positionOfMotor, double powerOfMotor){
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setTargetPosition(positionOfMotor);
        leftBackDrive.setTargetPosition(positionOfMotor);
        rightFrontDrive.setTargetPosition(positionOfMotor);
        rightBackDrive.setTargetPosition(positionOfMotor);

        leftBackDrive.setPower(powerOfMotor);
        leftFrontDrive.setPower(powerOfMotor);
        rightBackDrive.setPower(powerOfMotor);
        rightFrontDrive.setPower(powerOfMotor);

        while (opModeIsActive() && ((leftFrontDrive.getCurrentPosition()+rightBackDrive.getCurrentPosition()+rightFrontDrive.getCurrentPosition()+leftBackDrive.getCurrentPosition())/4) < positionOfMotor){
            telemetry.addData("Left Front Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("Left Back Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("Right Front Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("Right Back Position", rightBackDrive.getCurrentPosition());
            telemetry.addData("Power", powerOfMotor);
            telemetry.addData("Target Position", positionOfMotor);
            telemetry.update();
            }

        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    public void IMUTurning(double leftPower, double rightPower, double target){
        double currentValue = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
        double realTarget = currentValue + target;

        useEncoders();

        if (currentValue <= realTarget) {

            leftBackDrive.setPower(leftPower);
            leftFrontDrive.setPower(leftPower);
            rightBackDrive.setPower(rightPower);
            rightFrontDrive.setPower(rightPower);

            while (opModeIsActive() && currentValue<realTarget){
                telemetry.addData("LeftPower", leftPower);
                telemetry.addData("RightPower", rightPower);
                telemetry.addData("Z axis", imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("Y axis", imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
                telemetry.addData("X axis", imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);
                telemetry.addData("Current Value", currentValue);
                telemetry.update();
                currentValue = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
            }
        }
        else if (currentValue>realTarget){

            leftBackDrive.setPower(leftPower);
            leftFrontDrive.setPower(leftPower);
            rightBackDrive.setPower(rightPower);
            rightFrontDrive.setPower(rightPower);

            while (opModeIsActive() && currentValue>realTarget){
                telemetry.addData("LeftPower", leftPower);
                telemetry.addData("RightPower", rightPower);
                telemetry.addData("Z axis", imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("Y axis", imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
                telemetry.addData("X axis", imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);
                telemetry.addData("Current Value", currentValue);
                telemetry.update();
                currentValue = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
            }

        }

        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }
}

package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    public CRServo dumper = null;
    public Servo intakeFlip1 = null;
    public Servo intakeFlip2 = null;

    public HardwareMap hwMap = null;


    public DistanceSensor dSensor1;

    public BNO055IMU imu;
    public Orientation angles;

    @Override
    public void runOpMode() {

    }

    //this method makes a motor run for a certain number of encoder ticks
    public void setMotorPosition(DcMotor yourMotor, int targetPosition, double speed, LinearOpMode om){
        yourMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yourMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        yourMotor.setTargetPosition(targetPosition);
        yourMotor.setPower(speed);

        //wait for the motor to be
        while (Math.abs(yourMotor.getCurrentPosition())<yourMotor.getTargetPosition()&&om.opModeIsActive()){
            om.telemetry.addData("Status", "not there yet");
            om.telemetry.addData("Position", yourMotor.getCurrentPosition());
            om.telemetry.addData("Target", yourMotor.getTargetPosition());
            om.telemetry.update();
        }

        yourMotor.setPower(0);
    }

    //lowers the robot
    public void lowering(LinearOpMode om){
        //turn the hanging motor
        hangingMotor.setPower(1);

        //wait until the distance sensor is less than 4.5 in from the ground

        while (dSensor1.getDistance(DistanceUnit.INCH)>4.5){
            om.telemetry.addData("Distance to ground", dSensor1.getDistance(DistanceUnit.INCH));
            om.telemetry.update();
        }

        hangingMotor.setPower(0);

        //use encoders to lower the robot completely (turn 3.5 rotations)
        setMotorPosition(hangingMotor, 1240, 0.7, om);
    }


    //initialize all the robot hardware
    public void initRobot(HardwareMap ahwmap){
        hwMap = ahwmap;
        dSensor1 = hwMap.get(DistanceSensor.class, "DistanceSensor1");
        leftBackDrive = hwMap.get(DcMotor.class, "BackLeft");
        rightBackDrive = hwMap.get(DcMotor.class, "BackRight");
        leftFrontDrive = hwMap.get(DcMotor.class, "FrontLeft");
        rightFrontDrive = hwMap.get(DcMotor.class, "FrontRight");
        hangingMotor = hwMap.get(DcMotor.class, "HangingArm");
        intake = hwMap.get(DcMotor.class, "Intake");
        intakeExtender = hwMap.get(DcMotor.class, "IntakeExtend");
        scoring = hwMap.get(DcMotor.class, "Scoring");
        intakeFlip1 = hwMap.get(Servo.class, "IntakeFlip1");
        intakeFlip2 = hwMap.get(Servo.class, "IntakeFlip2");
        dumper = hwMap.get(CRServo.class, "Dumper");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeFlip1.setDirection(Servo.Direction.FORWARD);
        intakeFlip2.setDirection(Servo.Direction.REVERSE);

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

    //this function makes the robot turn to an angle relative to the position that it was
    //at at the VERY beginning of the opmode
    public void reorientIMU(double targetAngle, double left, double right, double threshold,
                            LinearOpMode om, double kp, double ki, double kd) {
        //get the current value in radians
        double currentValue = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        //convert the target to radians
        targetAngle = Math.toRadians(targetAngle);
        //initialize PID variables
        double error;
        double derivative;
        double integral = 0;
        double lastError = 0;
        double output;
        //convert the threshold to radians
        threshold = Math.toRadians(threshold);
        useEncoders();
        while (Math.abs(targetAngle - currentValue) > threshold && om.opModeIsActive()) {
            //the error (aka proportional) is the difference between set point and current point
            error = targetAngle- currentValue;
            //integral is the summation of all the past error
            integral += error;
            //derivative is the difference between current and past error
            //tries to predict future error
            derivative = error - lastError;
            //multiply each value by their respective constants and sum to get outuput
            output = (error * kp) + (integral * ki) + (derivative * kd);

            //set motor power based output value
            leftFrontDrive.setPower(output * left);
            leftBackDrive.setPower(output * left);
            rightFrontDrive.setPower(output * right);
            rightBackDrive.setPower(output * right);

            //get the current value from the IMU
            currentValue = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
            om.telemetry.addData("Current Value", currentValue);
            om.telemetry.addData("Target", targetAngle);
            om.telemetry.addData("Left Power", leftBackDrive.getPower());
            om.telemetry.addData("Right Power", rightBackDrive.getPower());
            om.telemetry.update();
            //make the last error equal to the current error
            lastError = error;
        }
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

        om.sleep(sleepTime);

        stopDrivetrain();
    }

    public void IMUTurn(double target, double rightPower, double leftPower, double threshold,
                        LinearOpMode om, double kp, double ki, double kd){
        double currentValue = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        target = Math.toRadians(target);
        double realTarget = currentValue + target;
        double error;
        double derivative;
        double integral = 0;
        double lastError = 0;
        double output;
        threshold = Math.toRadians(threshold);
        useEncoders();


        while(Math.abs(realTarget-currentValue)>threshold && om.opModeIsActive()){
            error = realTarget-currentValue;
            integral += error;
            derivative = error - lastError;
            output = (error*kp)+(integral*ki)+(derivative*kd);

            leftFrontDrive.setPower(output*leftPower);
            leftBackDrive.setPower(output*leftPower);
            rightFrontDrive.setPower(output*rightPower);
            rightBackDrive.setPower(output*rightPower);

            currentValue = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
            om.telemetry.addData("Current Value", currentValue);
            om.telemetry.addData("Target", target);
            om.telemetry.addData("Left Power", leftBackDrive.getPower());
            om.telemetry.addData("Right Power", rightBackDrive.getPower());
            om.telemetry.update();
            lastError = error;
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

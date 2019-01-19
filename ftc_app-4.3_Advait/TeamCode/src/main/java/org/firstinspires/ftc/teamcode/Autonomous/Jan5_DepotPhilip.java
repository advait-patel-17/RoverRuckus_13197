package org.firstinspires.ftc.teamcode.Autonomous;
//Code for meet 2
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
public class Jan5_DepotPhilip extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor Lift = null;
    private DcMotor intakeflipper = null;
    private Servo teammarker = null;
    private CRServo arm = null;
    int position = 0;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = " ARZRAS7/////AAABmbR4NiNb100sgpkr9nmSScUvCym5yLTzQSDrYMiRekDT9FE+ytpdcIkFY+dthwLJbrqAxTiDri2E2/X63uT6Z5J8QNkxMj+TcdbkaIhns782IcXEjgK3tFXZH9fdeFtTFQDtCT8b1PMWBNrqOf4ab28/sTmOxGjzzk3YWj7QTZTLdieYbuwoxVN1WH4b8eql+QNuiDDglg8LLBt9HLc4XLOSOad913m5c8AQiXDYJoL0tPSK5JW3/b9+itGeN6t4/R5SF91avy5c7QqZaHaJ6X6k2LfeL1nd9JifHLzGVWjQ6bVPFr/Eb3QESDnIF/eF8Alb5jv01AOEcaQ/rexkGyXsZ011elpxAJOO5zM8hWqj ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized1");
        telemetry.update();
        leftBackDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BackRight");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        intakeflipper = hardwareMap.get(DcMotor.class, "IntakeFlipper");
        Lift = hardwareMap.get(DcMotor.class, "HangingArm");

        teammarker = hardwareMap.get(Servo.class,"TeamMarker");
        arm = hardwareMap.get(CRServo.class,"MineralFlipper");

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeflipper.setDirection(DcMotor.Direction.REVERSE);
        Lift.setDirection(DcMotor.Direction.FORWARD);
        intakeflipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPower(0);

        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        intakeflipper.setPower(0);
        Lift.setPower(0);

        initVuforia();
        imuinit();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        //All Initialization
        waitForStart();

        runtime.reset();
//537 ticks equals a rotation

        //Lift.setPower(1);
        //sleep(5100);
        //Lift.setPower(0);
        //Bring the Robot down to the ground
        strafinghorizontal(0.5,500);
        //bring the robot a bit to the right so that it isnt hooked onto the hook
        SetToPosition(500, 0.25);
        //Lander to sampling
        telemetry.addData("status","ready for turning");
        telemetry.update();
        IMUTurn(130,0.3, -0.3, 2);
        //turn to detect right two
        strafinghorizontal(-0.25,1100);
        //strafe back a bit
        timebasedstraight(-0.25,350);

        if (tfod != null) {
            tfod.activate();
        }
        int mineralposition=0;
        int counter = 0;

        while (mineralposition==0 && counter<4 && opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }

                        //moves robot backwards until it sees 2 minerals
                        if (goldMineralX != -1 && silverMineral1X != -1) {
                            if (goldMineralX < silverMineral1X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                mineralposition = 1;
                            } else if (goldMineralX > silverMineral1X) {
                                telemetry.addData("Gold Mineral Position", "Center");
                                mineralposition = 2;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Right");
                                mineralposition = 3;
                            }
                        }
                    }
                    telemetry.update();
                }
            }
            counter++;
            sleep(2000);
        }
        //timebasedstraight(0.25,130);
        //strafinghorizontal(0.25,1000);
        //turning(-130,0.3);
        switch(mineralposition) {
            case 1:
                telemetry.addData("sample","gold left");
                telemetry.update();

                //strafing(90,0.2,6000);
                //strafe to be in front of the gold mineral
                //SetToPosition(300,0.2);
                //drive forward to hit the gold mineral
                break;
            case 2:
                telemetry.addData("sample","gold middle");
                telemetry.update();
                //strafing(90,0.2,5000);
                //strafe to be in front of the gold mineral
                //SetToPosition(300,0.2);
                //drive forward to hit the gold mineral
                break;
            case 3:
                telemetry.addData("sample","gold right");
                telemetry.update();
                //strafing(90,0.2,3000);
                //strafe to be in front of the gold mineral
                //SetToPosition(300,0.2);
                //drive forward to hit the gold mineral
                break;
            default:
                // SetToPosition(600,0.2);
                //drive forward all the way into the crater
                break;
        }

        sleep(20000);
        if (tfod != null) {
            tfod.shutdown();
        }

    }

    public void timebasedstraight(double power, int time){
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        sleep(time);
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    public void IMUTurn(double target, double rightPower, double leftPower, double threshold){
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
            sleep(100);
        }

        stopDrivetrain();

    }


    public void imuinit() {
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
        imu = hardwareMap.get(BNO055IMU.class, "imu 2");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }
    public void SetToPosition(int ticks, double power){
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setTargetPosition(ticks);
        rightFrontDrive.setTargetPosition(ticks);
        leftBackDrive.setTargetPosition(ticks);
        rightBackDrive.setTargetPosition(ticks);

        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        while (opModeIsActive() && leftFrontDrive.getCurrentPosition() < leftFrontDrive.getTargetPosition()) {
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void initVuforia() {



        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void strafing(int angle, double power, int strafetime){
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double robotAngle = angle - Math.PI / 4;
        final double v1 = power * Math.cos(robotAngle);
        final double v2 = power * Math.sin(robotAngle);
        final double v3 = power * Math.sin(robotAngle);
        final double v4 = power * Math.cos(robotAngle);

        leftFrontDrive.setPower(v1);
        rightFrontDrive.setPower(v2);
        leftBackDrive.setPower(v3);
        rightBackDrive.setPower(v4);

        sleep(strafetime);

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    private void strafinghorizontal(double power, int strafetime){
        useEncoders();

        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(power);

        sleep(strafetime);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void useEncoders(){
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopDrivetrain(){
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
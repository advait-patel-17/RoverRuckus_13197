
package org.firstinspires.ftc.teamcode.Autonomous.PastAuto;

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

//import org.firstinspires.ftc.robotcore.external.ClassFactory;
/*import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
*/

//import java.util.List;

@Autonomous
@Disabled
public class Dec1_Crater extends LinearOpMode{

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

    ColorSensor sensorColorLeft;
    ColorSensor sensorColorRight;


    /*private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
*/
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    //private static final String VUFORIA_KEY = "ARZRAS7/////AAABmbR4NiNb100sgpkr9nmSScUvCym5yLTzQSDrYMiRekDT9FE+ytpdcIkFY+dthwLJbrqAxTiDri2E2/X63uT6Z5J8QNkxMj+TcdbkaIhns782IcXEjgK3tFXZH9fdeFtTFQDtCT8b1PMWBNrqOf4ab28/sTmOxGjzzk3YWj7QTZTLdieYbuwoxVN1WH4b8eql+QNuiDDglg8LLBt9HLc4XLOSOad913m5c8AQiXDYJoL0tPSK5JW3/b9+itGeN6t4/R5SF91avy5c7QqZaHaJ6X6k2LfeL1nd9JifHLzGVWjQ6bVPFr/Eb3QESDnIF/eF8Alb5jv01AOEcaQ/rexkGyXsZ011elpxAJOO5zM8hWqj\n";

    /*//{@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    //private VuforiaLocalizer vuforia;

    /*//{@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    //private TFObjectDetector tfod;


    @Override
    public void runOpMode() throws InterruptedException {
        //Init Dc motors
        initMotors();
        //init Servos
        initServos();
        //Color Sensors
        initColorSensors();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        //initVuforia();

        initIMU();

        /*if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }*/

        /* Wait for the game to begin */
        telemetry.addData("Status", "Initialized. Good to go!");
        telemetry.update();
        waitForStart();
        runtime.reset();
        //hangingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //hangingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangingMotor.setPower(1);
        sleep(5800);
        hangingMotor.setPower(0);

        sleep(500);
        //moveWithEncoders(0.2, 1000);

        strafe(0.5, 500);
        sleep(500);

        //moveWithEncoders(0.5, 400);

        //strafe(-0.3, 700);

        //hangingMotor.setPower(-1);
        //double time = getRuntime();

        IMUTurn(90, 0.5, -0.3, 5);

        /*while ((getRuntime()-time<4) && opModeIsActive()){
            telemetry.addData("Status", "Turn is finished");
            telemetry.addData("Status", "Arm Lowering");
            telemetry.addData("Time Left", getRuntime() - time);
            telemetry.update();
        }*/

        //hangingMotor.setPower(0);

        //moveWithEncoders(-0.2, 300);

        //sleep(300);

        strafe(0.3, 750);

        moveWithEncoders(-0.4, 690);

        sleep(500);

        samplingArm.setPosition(0.8);
        sleep(2000);

        int counter = 0;
        while (counter<3 && opModeIsActive()){
            telemetry.addData("Right Red", sensorColorRight.red());
            telemetry.addData("Right Green", sensorColorRight.green());
            telemetry.addData("Right Blue", sensorColorRight.blue());
            telemetry.addData("Left Red", sensorColorLeft.red());
            telemetry.addData("Left Green", sensorColorLeft.green());
            telemetry.addData("Left Blue", sensorColorLeft.blue());
            telemetry.update();
            sleep(100);
            counter++;
        }

        if (sensorColorRight.red()>60 && sensorColorRight.green()>60 && sensorColorRight.blue()<60){
            telemetry.addData("Position", 1);
            telemetry.update();
            moveWithEncoders(-0.2, 600);
            sleep(300);
            samplingArm.setPosition(0);
            moveWithEncoders(0.5, 1800);
            sleep(500);
        }
        else if (sensorColorLeft.red()>60 && sensorColorLeft.green()>60 && sensorColorLeft.blue()<60) {
            telemetry.addData("Position", 2);
            telemetry.update();
            moveWithEncoders(0.2, 600);
            sleep(500);
            samplingArm.setPosition(0);
            moveWithEncoders(0.5, 1000);
            sleep(500);
        }
        else {
            telemetry.addData("Position", 3);
            telemetry.update();
            samplingArm.setPosition(0);
                sleep(300);
                moveWithEncoders(0.5, 1000);
                sleep(500);
                strafe(0.2, 1500);
                strafe(-0.2, 1500);
                sleep(500);
        }
        moveWithEncoders(0.5,1300);
        sleep(500);
        IMUTurn(-135,-0.50,0.50,3);
        sleep(500);
        strafe(-0.5, 500);
        moveWithEncoders(-0.5,2500);
        sleep(500);
        teamMarker.setPosition(0.2);
        sleep(1300);
        teamMarker.setPosition(0.8);
        moveWithEncoders(0.8,2000);
        sleep(500);
        intakeFlipper.setPower(-0.5);
        sleep(750);
        intakeFlipper.setPower(0);
        sleep(20000);
        /*moveWithEncoders(0.5, 1500);
        IMUTurn(45, 0.5, 0, 3);
*/


/*
        if (opModeIsActive()) {
            // Activate Tensor Flow Object Detection.
            if (tfod != null) {
                tfod.activate();
            }
        }

        int mineralPosition = 0;
        int counter = 0;
        while (mineralPosition == 0 && opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {
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
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                mineralPosition = 1;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                mineralPosition = 3;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                mineralPosition = 2;
                            }
                        }
                    }
                    telemetry.addData("Counter", counter);
                    telemetry.update();
                    counter++;
                    sleep(500);
                }
            }
        }


        switch (mineralPosition){
            case 1 :
                telemetry.addData("Position", "Left");
                telemetry.update();

                moveWithEncoders(0.5, 500);
                sleep(500);

                strafe(0.5, 500);
                sleep(500);

                strafe(-0.5, 500);
                sleep(500);

                break;

            case 2:
                telemetry.addData("Position", "Center");
                telemetry.update();

                strafe(0.5, 1250);
                sleep(500);

                strafe(-0.5, 750);
                sleep(500);

                moveWithEncoders(0.5, 500);
                sleep(500);

                break;

            case 3:
                telemetry.addData("Position", "Right");
                telemetry.update();

                moveWithEncoders(-0.5, 500);
                sleep(500);

                strafe(0.5, 500);
                sleep(500);

                strafe(-0.5, 500);

                moveWithEncoders(0.5, 1000);

                break;

            default:
                telemetry.addData("Position", "None Detected. Defaulting Center.");
                telemetry.update();

                strafe(0.5, 1250);
                sleep(500);

                strafe(-0.5, 750);
                sleep(500);

                moveWithEncoders(0.5, 500);
                sleep(500);

        }*/
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


    public void initColorSensors(){
        // get a reference to the color sensor.
        sensorColorRight = hardwareMap.get(ColorSensor.class, "SensorRight");

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attenuate the measured values.
        final double SCALE_FACTOR = 255;

        //get a reference to the color sensor.
        sensorColorLeft = hardwareMap.get(ColorSensor.class, "SensorLeft");
    }

    public void initServos(){
        samplingArm = hardwareMap.get(Servo.class, "Sample");
        teamMarker = hardwareMap.get(Servo.class, "TeamMarker");

        samplingArm.setPosition(0);
    }

/*    private void initVuforia() {

        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

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
*/
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

        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
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

package org.firstinspires.ftc.teamcode.RoverRuckusOnSeason.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


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

@TeleOp(name="FieldCentric", group="Linear Opmode")
@Disabled
public class Jan5_FieldCentricWithOperators extends LinearOpMode {

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

    BNO055IMU imu;
    Orientation angles;


    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        initMotors();
        initIMU();

        double shift = 1;



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //set this variable to the imu position
            double fieldAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
            //r = the distance of the joystick from the center
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            //uses arctangent to calculate the angle of the joystick
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI / 4) - fieldAngle;
            //turning is right joystick x axis
            double rightX = gamepad1.right_stick_x;

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
            hangingMotor.setPower(gamepad1.left_trigger-gamepad1.right_trigger);

            if (gamepad2.dpad_up){
                intakeExtender.setPower(1);
            }
            else if (gamepad2.dpad_down){
                intakeExtender.setPower(-1);
            }
            else {
                intakeExtender.setPower(0);
            }

            if (gamepad2.a){
                dumper.setPosition(0);
            }
            else if (gamepad2.b){
                dumper.setPosition(0.2);
            }

            else if (gamepad2.x){
                dumper.setPosition(0.6);
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
        imu = hardwareMap.get(BNO055IMU.class, "imu 2");
        imu.initialize(parameters);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }


}

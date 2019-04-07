package org.firstinspires.ftc.teamcode.RoverRuckusOnSeason.Autonomous.CurrentAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoverRuckusOnSeason.Robot.RobotV2;
import org.firstinspires.ftc.teamcode.RoverRuckusOnSeason.Robot.Vision;

@Autonomous
//@Disabled
public class Bradley_Crater extends LinearOpMode{


    RobotV2 robot = new RobotV2();
    Vision vision = new Vision();



    @Override
    public void runOpMode(){

        //Initialize hardware
        robot.initRobot(hardwareMap);
        //Initialize vision (Vuforia and TensorFlow)
        vision.initVision(hardwareMap);

        telemetry.addData("Status", "All set");
        telemetry.update();
        //Wait for Driver to press PLAY
        waitForStart();

        robot.intakeFlip1.setPosition(0.85);


        //Lower Down from latch
        robot.lowering(this);

        //        sleep(300);


        //strafe out of latch
        robot.strafe(0.5, 500, this);


        //back up against lander to straighten out
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setPower(-0.3);
        robot.leftFrontDrive.setPower(-0.3);
        sleep(300);
        robot.stopDrivetrain();

        //activate tensorflow
        if (opModeIsActive()) {
            // Activate Tensor Flow Object Detection.
            if (vision.tfod != null) {
                vision.tfod.activate();
            }
        }


        //set intake servos to a certain position so they don't come down
        //while the robot moves


        //go forward
        robot.moveWithEncoders(0.5, 400, this);


        robot.reorientIMU(0, -0.3, 0.3, 0.5, this, 0.9, 0.05, 0);

        //line up with the right mineral
        robot.strafe(0.5, 400, this);


        //bring lead screw down
        //robot.hangingMotor.setPower(-1);
        //go forward just a little bit more

        //bring down servo halfway first so the team marker doesn't fly out

        //stop bringing lead screw down
        //robot.hangingMotor.setPower(0);

        //check if the mineral is gold
        boolean isGold = vision.checkForGold(this);
        telemetry.addData("Gold?", isGold);
        telemetry.update();
        //if gold has been detected
        if (isGold){
            //line up to the mineral
            robot.strafe(0.5, 550, this);

            //push it off of the square
            robot.moveWithEncoders(0.5, 600, this);

            //come back
            robot.moveWithEncoders(-0.3, 800, this);

            //turn 90 deg using the PID algorithm
            robot.reorientIMU(90, -0.3, 0.3, 1, this, 1.1, 0.02, 0.025);
            //strafe over just a little so that you are out of the sampling field
            robot.strafe(0.3, 400, this);
            //go to the right mineral
            robot.moveWithEncoders(0.5, 1000, this);
        }

        //if no gold
        else {
            //go to next (center) mineral
            robot.strafe(-0.5, 800, this);
            // check if it is gold
            isGold = vision.checkForGold(this);
            telemetry.addData("Gold?", isGold);
            telemetry.update();
            // it is gold
            if (isGold){
                //strafe over to it
                robot.strafe(0.5, 300, this);

                // push it off of the square
                robot.moveWithEncoders(0.5, 600, this);
                //come back
                robot.moveWithEncoders(-0.3, 800, this);

                //turn 90 degrees
                robot.reorientIMU(90, -0.3, 0.3, 1, this,
                        1.1, 0.02, 0.025);

                //go to third mineral
                robot.moveWithEncoders(0.5, 500, this);
            }
            // if the center mineral isn't gold, then the gold must be at
            //the right position
            else{
                telemetry.addData("Gold?", "default right");
                telemetry.update();

                //go to the right mineral
                robot.strafe(-0.5, 800, this);

                //move the mineral off of its square
                robot.moveWithEncoders(0.5, 600, this);

                //come back
                robot.moveWithEncoders(-0.3, 800, this);

                //turn 90 degrees
                robot.reorientIMU(90, -0.3, 0.3, 1, this,
                        1.1, 0.02, 0.025);
            }
        }

        //go to the wall
        robot.moveWithEncoders(0.7, 800, this);
        // turn towards the depot
        robot.reorientIMU(135, -0.5, 0.5, 0.5, this,
                0.9, 0, 0);

        //strafe into the wall to square up
        robot.strafe(0.6, 1200, this);
        robot.strafe(-0.5, 300, this);
        sleep(100);

        // go to the depot
        robot.moveWithEncoders(0.5, 950, this);
        sleep(100);
        //bring down intake
        robot.intakeFlip1.setPosition(0.65);

        sleep(200);
        robot.intakeFlip1.setPosition(0.55);
        sleep(200);
        robot.intakeFlip1.setPosition(0.45);

        //outtake team marker
        robot.intake.setPower(1);
        sleep(1200);
        robot.intake.setPower(0);

        //bring intake back up
        robot.intakeFlip1.setPosition(0.85);

        //move backwards towards the crater
        robot.moveWithEncoders(-0.7, 1300, this);

        sleep(1000);
    }
}
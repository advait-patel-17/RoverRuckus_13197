package org.firstinspires.ftc.teamcode.Autonomous.PastAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Vision;

@Autonomous
@Disabled
public class RegionalDepot extends LinearOpMode{

    // call the hardware from the Robot class
    Robot robot = new Robot();
    //call the objects from the Vision class
    Vision vision = new Vision();

    //Having all of our hardware and functions in a separate class allows us
    //to more easily organize our code. This way, whenever we need to use a function,
    //we can just call them from there instead of cluttering up the code in this program

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


        //Lower Down from latch
        robot.lowering(this);
//        sleep(100);



        //strafe out of latch
        robot.strafe(0.5, 450, this);
        sleep(100);

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
        robot.intakeFlip1.setPosition(0.73);
        robot.intakeFlip2.setPosition(0.73);



        //go forward
        robot.moveWithEncoders(0.5, 600, this);
        sleep(100);

        //strafe over a little past the silver mineral
        robot.strafe(-0.5, 500, this);
        sleep(100);

        //bring lead screw down
        //robot.hangingMotor.setPower(-1);
        //go forward just a little bit more
        robot.moveWithEncoders(0.3, 300, this);

        robot.reorientIMU(0, -0.3, 0.3, 0.5, this, 0.9, 0.02, 0.008);

        //extend intake to put team marker in depot
        robot.intakeExtender.setPower(1);
        sleep(1350);
        robot.intakeExtender.setPower(0);

        //bring down servo halfway first so the team marker doesn't fly out
        robot.intakeFlip1.setPosition(0.55);
        robot.intakeFlip2.setPosition(0.55);

        sleep(300);

        //bring intake down all the way
        robot.intakeFlip1.setPosition(0.3);
        robot.intakeFlip2.setPosition(0.3);
        sleep(100);

        //outtake team marker
        robot.intake.setPower(-1);
        sleep(1300);
        robot.intake.setPower(0);

        //stop bringing lead screw down
        //robot.hangingMotor.setPower(0);

        //bring intake back up
        robot.intakeFlip1.setPosition(0.73);
        robot.intakeFlip2.setPosition(0.73);

        //retract intake
        robot.intakeExtender.setPower(-1);
        sleep(1000);
        sleep(350);
        robot.intakeExtender.setPower(0);
        sleep(100);

        //move back a little
        robot.moveWithEncoders(-0.3, 400, this);
        sleep(100);

        //strafe over to the right mineral
        robot.strafe(0.5, 400, this);

        boolean isGold = vision.checkForGold(this);
        telemetry.addData("Gold?", isGold);
        telemetry.update();
        if (isGold){
            // line up with the gold
            robot.strafe(0.5, 400, this);
            sleep(100);
            robot.moveWithEncoders(-0.3, 300, this);

 //           robot.reorientIMU(0, -0.3, 0.3, 0.5, this, 0.9, 0.05, 0);
            // bring down intake
            robot.intakeFlip1.setPosition(0.3);
            robot.intakeFlip2.setPosition(0.3);
            robot.intake.setPower(1);
            // move forward to intake gold
            robot.moveWithEncoders(0.3, 700, this);
            sleep(500);

            robot.intake.setPower(0);
            sleep(100);

            //bring intake back up
            robot.intakeFlip1.setPosition(0.75);
            robot.intakeFlip2.setPosition(0.75);


            robot.moveWithEncoders(-0.3, 600, this);

            //robot.moveWithEncoders(-0.5, 400, this);
            //use a PID algorithm to turn 90 deg towards the crater
            robot.reorientIMU(90, -0.3, 0.3, 0.05, this, 0.9, 0.01, 0.025);
            sleep(100);
            robot.moveWithEncoders(0.5, 500, this);
        }

        else {
            //line up the camera with the center mineral
            robot.strafe(0.5, 800, this);
            //check to see if it is the gold
            isGold = vision.checkForGold(this);
            if (isGold){
                //line up the intake with the mineral
                robot.strafe(0.5, 400, this);
                robot.moveWithEncoders(-0.3, 300, this);
                //robot.reorientIMU(0, -0.3, 0.3, 0.5, this, 0.9, 0.05, 0);

                //bring intake down
                robot.intakeFlip1.setPosition(0.3);
                robot.intakeFlip2.setPosition(0.3);
                robot.intake.setPower(1);

                //move forward to intake gold
                robot.moveWithEncoders(0.3, 700, this);
                sleep(500);


                robot.intake.setPower(0);
                sleep(100);

                //bring intake back up
                robot.intakeFlip1.setPosition(0.75);
                robot.intakeFlip2.setPosition(0.75);
                sleep(100);


                robot.moveWithEncoders(-0.3, 600, this);

                //use a PID algorithm to turn 90 deg towards the crater
                robot.reorientIMU(90, -0.3, 0.3, 0.5, this, 0.9, 0.01, 0.025);
                sleep(100);
                robot.moveWithEncoders(0.5, 1000, this);
            }
            else{//if it's not the center or the right mineral, then it has to be the left one
                //line up with the left mineral
                robot.strafe(-0.5, 1400, this);

                robot.moveWithEncoders(-0.5, 300, this);
                sleep(100);

                //bring intake down
                robot.intakeFlip1.setPosition(0.3);
                robot.intakeFlip2.setPosition(0.3);
                robot.intake.setPower(1);

                //move forward to intake mineral
                robot.moveWithEncoders(0.3, 700, this);
                sleep(500);

                robot.intake.setPower(0);
                sleep(100);

                //bring intake back up
                robot.intakeFlip1.setPosition(0.75);
                robot.intakeFlip2.setPosition(0.75);
                sleep(100);

                //robot.strafe(0.5, 500, this);
                robot.moveWithEncoders(-0.3, 600, this);
                //use a PID algorithm to turn 90 deg towards the crater
                robot.reorientIMU(90, -0.3, 0.3, 0.5, this,
                        0.9, 0.01, 0.025);
            }
        }

        //go into crater and bring down intake
        robot.moveWithEncoders(0.6, 1400, this);
        robot.intakeFlip1.setPosition(0.5);
        robot.intakeFlip2.setPosition(0.5);
        sleep(20000);
    }
}
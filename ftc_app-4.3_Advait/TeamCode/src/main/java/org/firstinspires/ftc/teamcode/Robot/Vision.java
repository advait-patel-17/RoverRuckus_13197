package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Disabled
public class Vision extends LinearOpMode{

    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    //public static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    public HardwareMap hwMap = null;

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
    public static final String VUFORIA_KEY = "ARZRAS7/////AAABmbR4NiNb100sgpkr9nmSScUvCym5yLTzQSDrYMiRekDT9FE+ytpdcIkFY+dthwLJbrqAxTiDri2E2/X63uT6Z5J8QNkxMj+TcdbkaIhns782IcXEjgK3tFXZH9fdeFtTFQDtCT8b1PMWBNrqOf4ab28/sTmOxGjzzk3YWj7QTZTLdieYbuwoxVN1WH4b8eql+QNuiDDglg8LLBt9HLc4XLOSOad913m5c8AQiXDYJoL0tPSK5JW3/b9+itGeN6t4/R5SF91avy5c7QqZaHaJ6X6k2LfeL1nd9JifHLzGVWjQ6bVPFr/Eb3QESDnIF/eF8Alb5jv01AOEcaQ/rexkGyXsZ011elpxAJOO5zM8hWqj\n";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;

    public void runOpMode(){

    }

    public void initVision(HardwareMap ahwMap){
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(ahwMap);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod(HardwareMap ahwMap) {
        hwMap = ahwMap;
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL);
    }

    public boolean checkForGold(LinearOpMode om){

        boolean mineralPosition = false;
        double startTime = om.getRuntime();
        while (om.opModeIsActive() && om.getRuntime()-startTime<1.5 && !mineralPosition) {
            if (tfod != null) {
                //get readings from the phone camera
                List<Recognition> recog = getGold();
                if (recog != null){
                    //for each mineral detected
                    for (Recognition r : recog){
                        //if it is gold and it is below a certain line on the camera,
                        //confirm that it is gold
                        if (r.getLabel().equals(LABEL_GOLD_MINERAL) && r.getBottom()>700){
                            mineralPosition = true;
                        }
                    }
                }
            }
        }
        return mineralPosition;
    }

    public double degreesToGold(LinearOpMode om){
        if (tfod != null) {
            //boolean foundGold = false;
            double startTime = om.getRuntime();
            double lowestY = -1;
            double centerX = 320;
            while(om.opModeIsActive() && om.getRuntime()-startTime < 1.5){
                List<Recognition> recog = getGold();
                if(recog != null) {
                    for (Recognition r : recog) {
                        //om.telemetry.addData("GoldLeft" , r.getLeft());
                        //om.telemetry.addData("GoldRight", r.getRight());
                        //om.telemetry.addData("GoldTop", r.getTop());
                        //om.telemetry.addData("GoldBottom", r.getBottom());
                        //om.telemetry.update();
                        if (r.getBottom() > 550) {
                            if (r.getBottom() > lowestY) {
                                lowestY = r.getBottom();
                                centerX = (r.getLeft() + r.getRight()) / 2;
                                //foundGold = true;
                            }
                        }
                    }
                }
            }
            return (centerX-320)*19/460;
        }
        else {
            return 0;}
    }


    public List<Recognition> getGold(){
        if(tfod != null){
            return tfod.getUpdatedRecognitions();
        }
        else{
            return null;
        }
    }

}

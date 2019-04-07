package org.firstinspires.ftc.teamcode.RoverRuckusOnSeason.SyntaxTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class MethodCalling {
    public static void main(String args[]){
        MyMethods test = new MyMethods();
        int a  = test.additionTest(1, 1);
        System.out.println(a);
    }
}

package org.firstinspires.ftc.teamcode.Auto.Detection;

public class COLOR {

    private int red = 0;
    private int green = 0;
    private int blue = 0;

    public COLOR(int r, int g, int b){
        red = r;
        green = g;
        blue=  b;
    }

    public void setRed(int r)
    {
        red = r;
    }
    public void setGreen(int g) {
        green = g;
    }
    public void setBlue(int b)
    {
        blue = b;
    }

    public int getRed(){
        return red;
    }
    public int getGreen(){
        return green;
    }
    public int getBlue(){
        return blue;
    }
    public int getBlack() {
        return 255 - getWhite(); //this makes a black value
    }
    public int getdBlue(){
        return -(green + red); // this makes a dark blue value
    }
    public int getWhite() {return ((green + blue + red) / (255 * 3)) * 255; }

    // Should probably create a getPrimaryColor() function and convert to HSV space to calculate

}

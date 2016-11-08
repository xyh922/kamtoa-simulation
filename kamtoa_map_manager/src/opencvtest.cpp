#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#define OCCUPIED 100
#define FREE 0
#define UNDEFINED -1

using namespace cv;
using namespace std;

cv::Mat originalImage;
cv::Mat rgbImg,original_rgbImg;
cv::Mat binaryImage;
cv::Mat binaryImage_original;
cv::Mat outputBin;
cv::Mat drawableAreaMask;
cv::Mat mask1;
cv::Mat mask2;
cv::Scalar roomColor;
int room_number;
char lastkey;
bool click;
int size;

void readFile(std::string path){
    originalImage   = cv::imread(path,CV_LOAD_IMAGE_UNCHANGED);
    rgbImg   = cv::imread(path,CV_LOAD_IMAGE_COLOR);
    original_rgbImg   = cv::imread(path,CV_LOAD_IMAGE_COLOR);
    // INT8 - Signed Char - Structure
    binaryImage     = cv::Mat(1024, 1024, CV_8SC1);
    outputBin       = cv::Mat(1024, 1024, CV_8UC1);
    // Create BinaryImage 
    uchar* mapDataIter = originalImage.data;
    // iterate from lower left 0,0 [ROW-MAJOR]
    for(unsigned int row = 0; row < binaryImage.rows; ++row){
        for(unsigned int col = 0; col < binaryImage.cols; ++col){
            if (*mapDataIter == 0)
                binaryImage.at<schar>(row,col) = 100;
            if (*mapDataIter == 254)
                binaryImage.at<schar>(row,col) = 0;
            if (*mapDataIter == 205)
                binaryImage.at<schar>(row,col) = -1;
            ++mapDataIter;
        }
    }
    binaryImage.copyTo(binaryImage_original);

    //Create Mask For Drawable Area 
    mask1 = (binaryImage == 100);
    mask2 = (binaryImage == -1);
    cv::bitwise_or(mask1,mask2,drawableAreaMask);
    // drawableAreaMask = (binaryImage == 0);
    // cv::bitwise_not(drawableAreaMask,drawableAreaMask);
}

void setBrush(int roomNum){
    switch(roomNum){
        case 0:
            roomColor = cv::Scalar(255,255,255);
            break;
        case 10:
            roomColor = cv::Scalar(255,0,0);
            break;
        case 20:
            roomColor = cv::Scalar(0,255,0);
            break;
        case 30:
            roomColor = cv::Scalar(0,0,255);
            break;
        case 40:
            roomColor = cv::Scalar(160,40,210);
            break;
        case 50:
            roomColor = cv::Scalar(255,175,130);
            break;
        case 60:
            roomColor = cv::Scalar(0,255,255);
            break;
        case 70:
            roomColor = cv::Scalar(255,255,0);
            break;
        case 80:
            roomColor = cv::Scalar(255,0,255);
            break;
        case 90:
            roomColor = cv::Scalar(255,225,255);
            break;

        default:
            roomColor = cv::Scalar(255,255,255);
        break;
    }
    std::cout << "Current Room Number : " << room_number <<std::endl;
}


void readValue(int x , int y){
    signed char reader = binaryImage.at<schar>(y,x);
    std::cout << (int)reader <<std::endl;
}

void createBinaryMat(){
    // Create BinaryImage 
    // iterate from lower left 0,0 [ROW-MAJOR]
    for(unsigned int row = 0; row < outputBin.rows; ++row){
        for(unsigned int col = 0; col < outputBin.cols; ++col){
            //cout << (int)binaryImage.at<schar>(row,col) <<endl;
            int readCell = (int)binaryImage.at<schar>(row,col);
            if(readCell == -1)outputBin.at<uchar>(row,col) = 205;
            else if(readCell == 100)outputBin.at<uchar>(row,col) =0;
            else if(readCell == 0)outputBin.at<uchar>(row,col) = 254;
            else outputBin.at<uchar>(row,col) = readCell;
        }
    }
}


void mouse_callback_draw(int event, int x, int y, int flags, void* userdata)
{
     int cell_value = (int)binaryImage.at<schar>(y,x);
     if  ( event == EVENT_LBUTTONDOWN )
     {
          click = true;            
     } 

     if (event == EVENT_MOUSEMOVE && click){
         //Drawable Region
          if(cell_value != -1 && cell_value != 100 && room_number != 0){
              //RGB - Draw by RGB Brush
              cv::rectangle(rgbImg,Point(x,y),Point(x+size,y+size),roomColor,-1);
              cv::bitwise_xor(rgbImg , rgbImg , rgbImg , drawableAreaMask );
              cv::bitwise_or(original_rgbImg,rgbImg,rgbImg,drawableAreaMask);
              //Binary - Draw by room_number 
              cv::rectangle(binaryImage,Point(x,y),Point(x+size,y+size),Scalar(room_number),-1);
              cv::bitwise_xor(binaryImage , binaryImage , binaryImage , drawableAreaMask );
              cv::bitwise_or(binaryImage_original,binaryImage,binaryImage,drawableAreaMask);
          }        
     }  

     if( event == EVENT_LBUTTONUP){
         click =false;
     }
}

int main( int argc, char** argv )
{
    //Open File by called arguments 
    // and put into mat
    readFile(argv[1]);
    click= false;
    size=10;
     //set the callback function for any mouse event
     namedWindow( "Original", WINDOW_AUTOSIZE );// Create a window for display.
     setMouseCallback("Original", mouse_callback_draw , NULL);

    // EDIT THE BINARY IMAGE
    // BUT DO THINGS ON RGB
    while(true){
        char key_input;
        if(key_input = cv::waitKey(1)){
            //std::cout << "Current Room Number : " << room_number <<std::endl;
        }
        if(key_input == 'q'){
            break;
        }
        if(key_input == '+')size+=10;
        if(key_input == '-'){
            size--;
            if(size <= 10)size=10;
        }
        
        if(key_input >= '0' && key_input <= '9'){
            room_number = key_input - '0';
            room_number *=10;
            //Set Brush Color
            setBrush(room_number);
        }
        imshow( "Original", rgbImg );       // Show our image inside it.  
    }
   
    createBinaryMat();
    imshow( "Bin", outputBin );
    imwrite( "fin.pgm", outputBin);
    waitKey(0);                                      // Wait for a keystroke in the window
    return 0;
}
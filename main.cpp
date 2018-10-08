
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include "scamp5d_spi.h"
#include "scamp5d_usb.h"
#include "scamp5d_proxy.h"
#include "vs_packet_decoder.hpp"
#include "debug_functions.h"
#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <pigpio.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/SVD"
#include "Eigen/LU"

using namespace Eigen;
using namespace std;


#ifdef RPI_VER
#define SPI_DEV_PATH        "/dev/spidev0.0"
#else
#define SPI_DEV_PATH        "/dev/spidev1.0"
#endif

#define ENABLE_PROXY


scamp5d_proxy *proxy;
scamp5d_interface*box;
vs_packet_decoder*packet_switch;

std::atomic<bool> quit(false);
bool host_on = false;
char filepath[256];
uint8_t bitmap24_buffer[256*256*3];


#define NUM_GPIO 4
#define ADJUST_PARA 150
#define MIN_WIDTH (1000 - ADJUST_PARA)
#define MAX_WIDTH (2000 + ADJUST_PARA)
#define ANGLE_RANGE 60.0 //rotatinig rangle
#define FREQUENCY 1000
#define MAX_EDGE 40

double center_x = -1.0;
double center_y = -1.0;
double center_y_beta = 0.0;
double center_y_beta_thre = 20;
double k_beta_y = 40;
double y_damp = 3;

double center_x_beta = 0.0;
double center_x_beta_thre = 20;
double k_beta_x = 40;
double x_damp = 3;

int init_servo = 0;
double servo_control_range = 0.0;
double resolution  = 0.0;
FILE *f;

struct patternCorner
{
   double x;
   double y;
};

patternCorner imageCorner[4];
patternCorner imageCornerOutput[4];

//record the sum length of the last pattern
double sum_length = 0;
double perimeter_threshold = 70;  // the reference perimeter is 105

//first order low pass filter coefficient
double alpha = 0.5; //(0,1)
double lastCenter_y = 256.0/2;
int cornerCount = 0; // used to record the four number of a pattern
int recordEffectiveImage = 0; //used to record number of frames that contain corners


int maxtrix_cal( MatrixXd P)
{

   //camera intrinsic matrix
   MatrixXd K(3,3);
   K(0,0) = 218.423;
   K(0,1) = 0;
   K(0,2) = 134.2604;
   K(1,0) = 0;
   K(1,1) = 218.4448;
   K(1,2) = 127.3721;
   K(2,0) = 0;
   K(2,1) = 0;
   K(2,2) = 1;

   MatrixXd Q(3,4);
   Q(0,0) = 33;
   Q(0,1) = -33;
   Q(0,2) = -33;
   Q(0,3) = 33;
   Q(1,0) = 33.75;
   Q(1,1) = 33.75;
   Q(1,2) = -33.75;
   Q(1,3) = -33.75;
   Q(2,0) = 1;
   Q(2,1) = 1;
   Q(2,2) = 1;
   Q(2,3) = 1;

   Matrix3d H = Matrix3d::Zero(3,3);

   MatrixXd Mit(3,3);
   Mit = Q*Q.transpose();

   H = K.inverse() * P * Q.transpose() * Mit.inverse();

   cout << H << endl;

  return 0;
}

int servo_control(double center_y)
{

  if(init_servo == 0)
  {
       if (gpioInitialise() < 0)
        return -1;
       gpioSetSignalFunc(SIGINT, 0);
       gpioSetPWMfrequency(NUM_GPIO,FREQUENCY);
       servo_control_range = MAX_WIDTH - MIN_WIDTH;
       resolution = ANGLE_RANGE/servo_control_range;
       printf("%f\n",resolution);

       //servo go to middle position
       gpioServo(NUM_GPIO,(MIN_WIDTH + MAX_WIDTH)/2.0);
       time_sleep(2);
       init_servo = 1;
  }

  if(center_y > 0)
  {
     double angle = center_y*ANGLE_RANGE/256.0;
     double servo_control_angle = MIN_WIDTH + angle/resolution;
     gpioServo(NUM_GPIO,servo_control_angle);
     //time_sleep(0.01);
  }

   return 0;
}


void setup_packet_switch(){

    // text are from the 'scamp5_send text' function
    packet_switch->case_text([&](const char*text,size_t length){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("packet[%d]: text, lc=%d: \n%s\n",sn,lc,text);
//        // report back some user values
//        box->post_message(VS_MSG_ROUTE_APP,VS_MSG_USER_VALUE,23,sn);
    });

    // boundingbox are from the 'scamp5_output_boundingbox' function
    packet_switch->case_boundingbox([&](const vs_array<uint8_t>&data)
    {
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("packet[%d]: aabb, lc=%d, { %d, %d, %d, %d }\n",sn,lc,data(0,0),data(0,1),data(1,0),data(1,1));

        int arthmetic_product = data(0,0)*data(0,1)*data(1,0)*data(1,1);


        if(arthmetic_product > 1) //return{ 0 0 0 0}, indicates there is no effective data obtained from the scamp
        {

            int center_tmp_x = (data(0,0) + data(1,0))/2.0;
            int center_tmp_y = (data(0,1) + data(1,1))/2.0;

             if(cornerCount <= 3)
            {
                imageCorner[cornerCount].x = center_tmp_x;
                imageCorner[cornerCount].y = center_tmp_y;
                cornerCount++;
            }
            else
            {
                MatrixXd Ip = MatrixXd::Zero(3,4);
                Ip(0,0) = imageCorner[0].x;
                Ip(0,1) = imageCorner[1].x;
                Ip(0,2) = imageCorner[2].x;
                Ip(0,3) = imageCorner[3].x;
                Ip(1,0) = 256.0 - imageCorner[0].y;
                Ip(1,1) = 256.0 - imageCorner[1].y;
                Ip(1,2) = 256.0 - imageCorner[2].y;
                Ip(1,3) = 256.0 - imageCorner[3].y;
                Ip(2,0) = 1.0;
                Ip(2,1) = 1.0;
                Ip(2,2) = 1.0;
                Ip(2,3) = 1.0;

                MatrixXd Ip_tmp = MatrixXd::Zero(3,4);
                Ip_tmp.row(0) = Ip.row(1);
                Ip_tmp.row(1) = Ip.row(0);
                Ip_tmp.row(2) << 1.0,1.0,1.0,1.0;

                maxtrix_cal( Ip_tmp );


                cornerCount = 0;
                double  patternCentreX = (imageCorner[0].x + imageCorner[1].x + imageCorner[2].x + imageCorner[3].x)/4;
                double  patternCentreY = (imageCorner[0].y + imageCorner[1].y + imageCorner[2].y + imageCorner[3].y)/4;

                for(int i = 0; i < 4; i++)
                {
                    double diffX = imageCorner[i].x - patternCentreX;
                    double diffY = imageCorner[i].y - patternCentreY;

                    if (diffX < 0 && diffY < 0)
                    {
                        imageCornerOutput[0] = imageCorner[i];
                    }
                    else if(diffX < 0 && diffY > 0)
                    {
                        imageCornerOutput[1] = imageCorner[i];
                    }
                    else if(diffX > 0 && diffY > 0)
                    {
                        imageCornerOutput[2] = imageCorner[i];
                    }
                    else if(diffX > 0 && diffY < 0)
                    {
                        imageCornerOutput[3] = imageCorner[i];
                    }
                    else
                    {
                        printf("Something maybe wrong");
                    };

                }

                //get rid of the wrong points
                double edge[4];

                edge[0] = pow(pow(imageCornerOutput[0].x - imageCornerOutput[1].x,2) + pow(imageCornerOutput[0].y - imageCornerOutput[1].y,2), 0.5);
                edge[1] = pow(pow(imageCornerOutput[1].x - imageCornerOutput[2].x,2) + pow(imageCornerOutput[1].y - imageCornerOutput[2].y,2), 0.5);
                edge[2] = pow(pow(imageCornerOutput[2].x - imageCornerOutput[3].x,2) + pow(imageCornerOutput[2].y - imageCornerOutput[3].y,2), 0.5);
                edge[3] = pow(pow(imageCornerOutput[3].x - imageCornerOutput[0].x,2) + pow(imageCornerOutput[3].y - imageCornerOutput[0].y,2), 0.5);

                int posnegflag = 1;
                for(int i = 0; i < 4; i++)
                {
                   if(edge[i] > MAX_EDGE)
                   {
                      posnegflag = -1;
                   }
                }
                // sorting the points in a given order
                // p1 topright p2 topleft

                //effective four points
                if(posnegflag > 0)
                {

                    //record the sum edge length
                    sum_length = edge[0] + edge[1] + edge[2] + edge[3];
                     //judge the changes of the shape
                    center_y_beta = imageCornerOutput[3].y + imageCornerOutput[2].y - imageCornerOutput[0].y - imageCornerOutput[1].y;
                    center_x_beta = imageCornerOutput[1].x + imageCornerOutput[2].x - imageCornerOutput[0].x - imageCornerOutput[3].x;

                    if(fabs(center_y_beta) > center_y_beta_thre)
                    {
                        center_y_beta = 0;
                    }
                     if(fabs(center_x_beta) > center_x_beta_thre)
                    {
                        center_x_beta = 0;
                    }

                    //k_beta_y = 0;
                    //k_beta_x = 0;

                    if(fabs(center_x_beta) >= 0.5 && fabs(center_y_beta) >= 0.5 )
                    {
                      center_y = patternCentreY + k_beta_y/(center_y_beta + y_damp * center_y_beta/fabs(center_y_beta));
                                                + k_beta_x/(center_x_beta + x_damp * center_x_beta/fabs(center_x_beta));
                    }
                    else if(fabs(center_y_beta) < 0.5 && fabs(center_x_beta) >= 0.5 )
                    {
                      center_y = patternCentreY + k_beta_x/(center_x_beta + x_damp * center_x_beta/fabs(center_x_beta));
                    }
                    else if(fabs(center_y_beta) >= 0.5 && fabs(center_x_beta) < 0.5 )
                    {
                      center_y = patternCentreY + k_beta_y/(center_y_beta + y_damp * center_y_beta/fabs(center_y_beta));
                    }
                    else
                    {
                      center_y = patternCentreY;
                    }

                    // record these points in txt for later analysis -- original data
                    for(int i = 0; i < 4; i++)
                    {
                     fprintf(f,"%d %f %f %f %f %f\n",recordEffectiveImage,imageCornerOutput[i].x,imageCornerOutput[i].y,center_y_beta,center_x_beta,center_y);
                    }
                    recordEffectiveImage++;

                    //first order low pass filter
                    double filter_center_y = alpha * center_y + (1-alpha)*lastCenter_y;
                    lastCenter_y = filter_center_y;
                    center_y = filter_center_y;

                }

            }

        }
        else
        {

           // the target is lost.
            if( sum_length <= perimeter_threshold)
            {
              // do not change rover direction when it is far away from the target
               center_x = -1.0;
               center_y = -1.0;
            }
            else
            {
               //when the rover gets close to the target and keep zero angle
               center_x = 128.0;
               center_y = 128.0;

               center_x_beta = 0;
               center_y_beta = 0;
            }

        }

        //printf("Box Center: x = %f, y = %f\n",center_x,center_y);

    });

    // data from the 'scamp5_output_analog' and the 'scamp5_output_digital' function
    packet_switch->case_analog([&](int width,int height,uint8_t*bitmap_buffer){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("packet[%d]: aout, lc=%d, width=%d, height=%d\n",sn,lc,width,height);
    });

    // data from the 'scamp5_output_analog' and the 'scamp5_output_digital' function
    packet_switch->case_digital([&](int width,int height,uint8_t*bitmap_buffer){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("packet[%d]: dout, lc=%d, width=%d, height=%d\n",sn,lc,width,height);
    });

    // ponits are data from the 'scamp5_output_events' function
    packet_switch->case_points([&](const vs_array<uint8_t>&data){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("packet[%d]: points, lc=%d, num_points=%d\n",sn,lc,data.get_row_size());
    });

    packet_switch->case_raw([&](const uint8_t*payload,size_t bytes){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("packet[%d]: raw, lc=%d, payload bytes=%d, [ %2.2X, %2.2X, %2.2X, %2.2X ... ]\n",sn,lc,bytes,
            payload[0],payload[1],payload[2],payload[3]);
    });

    packet_switch->case_data_int32([&](const vs_array<int32_t>&data){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        auto channel = packet_switch->get_data_channel();
        printf("packet[%d]: data array int32, lc=%d, int32[%d][%d], channel: %d\n",sn,lc,data.get_row_size(),data.get_col_size(),channel);
    });

    packet_switch->case_data_float([&](const vs_array<float>&data){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        auto channel = packet_switch->get_data_channel();
        printf("packet[%d]: data array float, lc=%d, int32[%d][%d], channel: %d\n",sn,lc,data.get_row_size(),data.get_col_size(),channel);
    });

}


void input_loop(){

    printf("<press Q to quit>\n");

    while(quit==false){
        char c = conio_getch();
        switch(c){

        case 'q':
            quit = true;
            printf("quit\n");
            break;

        case 's':// for diagnostic purpose
            printf("signature_counter: %d, packet_counter: %d\n",
            box->get_signature_counter(),box->get_packet_counter());
            break;

        case 'h':// manually toggle whether the device should behave as if the host app running
            if(host_on){
                box->post_message(VS_MSG_ROUTE_APP,VS_MSG_HOST_DC);
                printf("host off\n");
            }else{
                box->post_message(VS_MSG_ROUTE_APP,VS_MSG_HOST_ON);
                printf("host on\n");
            }
            host_on = !host_on;
            break;

        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
            printf("move first user slider: %d\n",c - '0');
            box->post_message(VS_MSG_ROUTE_APP,VS_MSG_GUI_UPDATE,VS_GUI_USER_ITEM_0,c - '0');
            break;
        }
    }
}


int main(int argc,char*argv[]){

    f = fopen("data recorded.txt","w");
    if(f == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }

    init_servo = 0;

    int r;

    printf("Scamp5d Application Example\n");

    // a thread to handle console keyboard input
    std::thread input_thread(input_loop);

    // 'scamp5d_packet_switch' the object to decode the incoming packets and pass them
    // to the corresponding handler function registered.
    packet_switch = new vs_packet_decoder;

    setup_packet_switch();

    /// Initialize the device interface
#if 0
    printf("SPI_DEV_PATH: %s\n",SPI_DEV_PATH);
    box = new scamp5d_spi();
    r = box->open(SPI_DEV_PATH,10000000);// for Odroid XU4 and Raspberry Pi 3, 10 MHz is the safe maximum speed
#else
    box = new scamp5d_usb();
    r = box->open("",0);
#endif
    if(r){
        fprintf(stderr,"<Error> failed to open device!\n");
        exit(-1);
    }else{
        printf("<Device Ready>\n------------------------------------------------\n");
    }


#ifdef ENABLE_PROXY

    /// Start the TCP Proxy
    proxy = new scamp5d_proxy;
    proxy->open("192.168.195.103",27725);

    /// Setup how the interface handles packet received from the device
    box->on_free_packet([&](uint8_t*packet,size_t packet_size){
        // Here the "free_packet" callback is used, which needs to 'free' the packet after use.
        // process the packet
        packet_switch->decode_packet(packet,packet_size,box->get_packet_counter());
        // broadcast through the proxy and free the packet
        proxy->broadcast(packet,packet_size,true);
    });

    /// Setup how the proxy handles packet received from client
    proxy->on_receive_packet([&](const uint8_t*packet,size_t bytes){
        //printf("client -> device: %d, %s\n",bytes,HEX_STR(packet,(bytes>16)? 16:bytes));
        printf("client -> device: %d bytes\n",bytes);
        box->write(packet,bytes);
    });

    //servo_control();
    /// Main Loop
//    double count_number = 0;
    while(quit==false)
    {
//        printf("counting loop times %f\n",count_number);
//        time_sleep(0.1);
        box->routine();
        proxy->routine();

        servo_control(center_y);

    }
    //close file
    fclose(f);

    //shutdown the servo
    gpioTerminate();
    proxy->close();
    delete proxy;

#else

    box->on_receive_packet([&](const uint8_t*packet,size_t packet_size){
        packet_switch->decode_packet(packet,packet_size,box->get_packet_counter());
    });

    while(quit==false){
        box->routine();
    }

#endif

    box->close();
    delete box;

    delete packet_switch;

    input_thread.join();

    return 0;
}

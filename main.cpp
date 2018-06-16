
#include <unistd.h>
#include <cstdlib>
#include "scamp5d_spi.h"
#include "scamp5d_usb.h"
#include "scamp5d_proxy.h"
#include "vs_packet_decoder.hpp"
#include "debug_functions.h"

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <pigpio.h>


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
#define MIN_WIDTH 1000
#define MAX_WIDTH 2000
#define ANGLE_RANGE 60.0 //rotatinig rangle
#define FREQUENCY 1000

double center_x = -1.0;
double center_y = -1.0;
int init_servo = 0;
double servo_control_range = 0.0;
double resolution  = 0.0;
FILE *f;

//first order low pass filter coefficient
double alpha = 0.5; //(0,1)
double lastCenter_y = 256.0/2;

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
     double angle = (256.0 - center_y)*ANGLE_RANGE/256.0;
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
    packet_switch->case_boundingbox([&](const vs_array<uint8_t>&data){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("packet[%d]: aabb, lc=%d, { %d, %d, %d, %d }\n",sn,lc,data(0,0),data(0,1),data(1,0),data(1,1));

        int arthmetic_product = data(0,0)*data(0,1)*data(1,0)*data(1,1);

        if(arthmetic_product > 1) //return{ 0 0 0 0}, indicates there is no effective data obtained from the scamp
        {

            center_x = (data(0,0) + data(1,0))/2.0;
            center_y = (data(0,1) + data(1,1))/2.0;

            //first order low pass filter
            double filter_center_y = alpha * center_y + (1-alpha)*lastCenter_y;
            lastCenter_y = filter_center_y;

            center_y = filter_center_y;

        }
        else
        {
            center_x = -1.0;
            center_y = -1.0;

        }

        printf("Box Center: x = %f, y = %f\n",center_x,center_y);
        fprintf(f,"Center Y: %f\n",center_y);

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

    f = fopen("record_center_y.txt","w");
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
//        count_number++;

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

/*
 * Scamp5d M0 Example 8 - Concentric Circle Tracking
 *
 */

#include <scamp5.hpp>

using namespace SCAMP5_PE;


#define DEFAULT_FRAMERATE	200
#define DEFAULT_FRAMEGAIN	3


// Global Variables
uint32_t framerate;
volatile int readout_mode;
volatile int threshold;
volatile int show_time;
volatile int show_boxcenter;
vs_stopwatch stopwatch;

volatile int dog_threshold;
volatile int dog_diffuse;
volatile int hdr_iterations;
volatile int hdr_gain;

volatile int shrink_size;

// Stand-alone Kernels

// Stand-alone Kernels
scamp5_kernel shrink_image([]{

	using namespace scamp5_kernel_api;

	DNEWS(R9, R5, east, 0);
	DNEWS(R8, R5, north, 0);
	DNEWS(R7, R5, south, 0);
	DNEWS(R6, R5, west, 0);
	AND(R9, R9, R8);
	AND(R9, R9, R7);
	AND(R5, R9, R6);
});


scamp5_kernel gain_x2_C([]{

	using namespace scamp5_kernel_api;

	// double
	bus(A,C);
	bus(B,C);
	bus(C,A,B);

	// saturate
	bus(A,C,IN);
	where(A);
		mov(C,IN);
	all();

});


// Difference of Gaussians Algorithm

void algo_DoG_C_to_R6(int diffuse,int threshold){

	// negate C into D
	scamp5_kernel_begin();
		bus(D,C);
	scamp5_kernel_end();

	// diffuse image in D
    scamp5_diffuse(D,diffuse);

    // compare image in C with diffused one in D, with threshold
	scamp5_in(E,threshold);
    scamp5_kernel_begin();
    	bus(B,D,C);
    	sub(D,B,E);
    	where(D);
    		MOV(R6,FLAG);
        all();
        OR(R5,R6);// R5 = R5 OR R6
    scamp5_kernel_end();

}

// HDR Difference of Gaussians

void HDR_DoG_R5(){

    /*
     * First Exposure: Exposure Time = Frame Loop Time
     */
    scamp5_kernel_begin();
    	get_image(C);// get half-scale image and reset PIX
        respix(F);// store reset level of PIX in F
    	CLR(R5);
    scamp5_kernel_end();

    // apply gain on C
    auto frame_gain = vs_gui_read_slider(VS_GUI_FRAME_GAIN);
    scamp5_load_in(125);// saturate at 125
    for(int i=1;i<frame_gain;i++){
        scamp5_launch_kernel(gain_x2_C);
    }

    // run the algorithm
    algo_DoG_C_to_R6(dog_diffuse,dog_threshold);

    /*
     * HDR Exposures: Exposure Time = [20, 60, 120, 200, 300, 420, 560, 720, ... ] microseconds
     */
    for(int i=0;i<hdr_iterations;i++){

        vs_wait(20 + 20*i);// time delay

    	scamp5_kernel_begin();
            getpix(C,F);// get half-scale image while keep PIX (require PIX reset level supplied)
        scamp5_kernel_end();

        // apply gain on C
        scamp5_load_in(125);// saturate at 125
        for(int i=1;i<hdr_gain;i++){
            scamp5_launch_kernel(gain_x2_C);
        }

        // run the algorithm
        algo_DoG_C_to_R6(dog_diffuse,dog_threshold);

    }
    // final result is in R5

}

int main(){

   // M0 initialization
	vs_init();

   // design the layout of the interface
    auto display_1 = vs_gui_add_display("point 1",0,0);
    auto display_2 = vs_gui_add_display("point 2",0,1);
    auto display_3 = vs_gui_add_display("point 3",1,0);
    auto display_4 = vs_gui_add_display("point 4",1,1);

    auto iteration_times = vs_gui_add_slider("iteration times: ",0,10,4);
    auto slider_ext_loop = vs_gui_add_slider("over iterations: ",0,10,6);
    vs_gui_add_slider("threshold: ",-120,120,-59,&threshold);
    vs_gui_add_switch("display timing: ",false,&show_time);
    vs_gui_add_switch("box center: ",false,&show_boxcenter);
    vs_gui_add_slider_latched("readout: ",0,3,1,&readout_mode);

    vs_gui_add_slider("HDR iteration: ",1,50,28,&hdr_iterations);
    vs_gui_add_slider("HDR gain: ",1,5,5,&hdr_gain);
    vs_gui_add_slider("DoG threshold: ",0,100,47,&dog_threshold);
    vs_gui_add_slider("DoG diffuse: ",1,10,6,&dog_diffuse);
    vs_gui_add_slider("Shrink Size: ",0,5,1,&shrink_size);

    vs_on_gui_update(VS_GUI_FRAME_RATE,[&](int32_t new_value){
        framerate = new_value;
        if(framerate > 0){
            vs_frame_trigger_set(1,framerate);
            vs_enable_frame_trigger();
            vs_post_text("frame trigger: 1/%d\n",(int)framerate);
        }else{
            vs_disable_frame_trigger();
            vs_post_text("frame trigger disabled\n");
        }
    });

    vs_on_host_connect([&](){
        vs_post_text("\nScamp5d M0 - Object tracking, the frame rate here is fixed to 500\n");
        vs_led_on(VS_LED_2);
        vs_gui_move_slider(VS_GUI_FRAME_RATE,DEFAULT_FRAMERATE);
        vs_gui_move_slider(VS_GUI_FRAME_GAIN,DEFAULT_FRAMEGAIN);
    });

    vs_on_host_disconnect([&](){
        vs_led_off(VS_LED_2);
        vs_frame_trigger_set(1,DEFAULT_FRAMERATE);
        show_time = false;
        show_boxcenter = false;
        readout_mode = 0;
    });

	// Setup Frame-rate
    vs_frame_trigger_set(1,DEFAULT_FRAMERATE);

    // Frame loop
    while(1)
    {
    	// sync with frame trigger timing
        vs_wait_frame_trigger();
    	// process any message received, and execute callback functions
        vs_process_message();

        //start to record image processing time
        stopwatch.reset();

        HDR_DoG_R5();
        scamp5_kernel_begin();
        	MOV(R6,R5);
        	MOV(R8,R5);
    		CLR(R7);
        scamp5_kernel_end();

        // capture image with a gain
//        int gain = vs_gui_read_slider(VS_GUI_FRAME_GAIN);
//        scamp5_get_image(C,B,gain);
//        // threshold image
//        scamp5_in(E,threshold);
//        scamp5_kernel_begin();
//            all();
//            CLR(R5,R6,R7);
//            sub(A,C,E);
//            where(A);
//            MOV(R5,FLAG);
//            MOV(R6,FLAG);
//            MOV(R8,FLAG);
//            all();
//		scamp5_kernel_end();

		//thresholding time
		auto t_thre = stopwatch.get_usec();

		stopwatch.reset();
		//iteration times
	   int i = vs_gui_read_slider(iteration_times);

	   while(i--)
	   {
			scamp5_flood(R7,R6,1);
			scamp5_kernel_begin();
				CLR_IF(R6,R7); // R6&~R7
				MOV(R5,R6);
				NOT(R6,R5);
			scamp5_kernel_end();
	   }

       // extended iterations to increase the tracking robustness
       int j = 0;
       while(j < vs_gui_read_slider(slider_ext_loop)){
       	scamp5_flood(R7,R6,1);
           scamp5_kernel_begin();
           	MOV(R9,R5);// backup R5(to retrieve in case of nothing left)
       		CLR_IF(R6,R7);
       		MOV(R5,R6);
       		NOT(R6,R5);
           scamp5_kernel_end();

           // if there is nothing left
           if(!scamp5_global_or(R5)){
            	// retrieve R5 if after this iteration there is nothing left
                scamp5_kernel_begin();
            		MOV(R5,R9);
                scamp5_kernel_end();
                break;
            }
           j++;
       }


        //get rid of noise by shrinking the image
    	//shrink images to get rid of small noise
   	    for(int i=0; i<shrink_size; i++)
   	    {
   	        scamp5_launch_kernel(shrink_image);
   	    }

       // the results were stored in R5
       scamp5_kernel_begin();
          MOV(R7,R5);
          MOV(R6,R5);
          CLR(R10);
          CLR(R9);
          CLR(R5);
          CLR(R11);
  	   scamp5_kernel_end();

		// Blob Extraction Iterative Algorithm
		int count = 0;
		uint8_t event_coords[8][2];
		while(scamp5_global_or(R6))
		{

			// get first '1' in R6 image
			scamp5_scan_events(R6,event_coords[count],1);

			// put the pixel into R7 and flood using R6 as mask
			scamp5_load_point(R7,event_coords[count][1],event_coords[count][0]);
			scamp5_flood(R7,R6,0);

			// R7 now contain the found blob only.
			// use XOR to remove the blob from R6
	        scamp5_kernel_begin();
	        	XOR(R8,R6,R7);
	        	MOV(R6,R8);
			scamp5_kernel_end();

			if(count == 0)
			{
				scamp5_kernel_begin();
					MOV(R10,R7);
				scamp5_kernel_end();
			}
			if(count == 1)
			{
				scamp5_kernel_begin();
					MOV(R9,R7);
				scamp5_kernel_end();
			}
			if(count == 2)
			{
				scamp5_kernel_begin();
					MOV(R5,R7);
				scamp5_kernel_end();
			}
			if(count == 3)
			{
				scamp5_kernel_begin();
					MOV(R11,R7);
				scamp5_kernel_end();
			}

			count++;
			if(count>=8){
				break;
			}
		}

		uint8_t aabb_coords_1[4];
		uint8_t aabb_coords_2[4];
		uint8_t aabb_coords_3[4];
		uint8_t aabb_coords_4[4];

		scamp5_output_boundingbox(R10,display_1,aabb_coords_1);
		scamp5_output_boundingbox(R9,display_2,aabb_coords_2);
		scamp5_output_boundingbox(R5,display_3,aabb_coords_3);
		scamp5_output_boundingbox(R11,display_4,aabb_coords_4);

        //scamp5_output_image(C,display_1);


	 //  auto t_flood = stopwatch.get_usec();

	  // scamp5_output_image(R6,display_3);

	   //////////////////////////////////////////////
	   //output
	  // uint8_t aabb_coords[4];
	 //  scamp5_output_boundingbox(R5,display_4,aabb_coords);

//	   if(readout_mode>=1)
//	   {
//		  scamp5_output_image(R5,display_3);
//	   }
//	   if(readout_mode>=2)
//	   {
//		   scamp5_output_image(R8,display_2);
//	   }
//	   if(readout_mode>=3)
//	   {
		   //scamp5_output_image(C,display_1);
//	   }

	   //caculate the center of the bounding box(object center)
//	   auto centerX = (aabb_coords[0] + aabb_coords[2])/2;
//	   auto centerY = (aabb_coords[1] + aabb_coords[3])/2;
//
//	   //if there is nothing detected
//       if(!scamp5_global_or(R5))
//       {
//    	   centerX = 0;
//    	   centerY = 0;
//       }
//
//		if(show_time)
//		{
//			if(vs_loop_counter_get()%5==0)
//			{
//				vs_post_text("Time(us): Threshold=%d; Flood=%d; Sum=%d\n",t_thre,t_flood,(t_thre+t_flood));
//			}
//		}
//
//		if(show_boxcenter)
//		{
//			if(vs_loop_counter_get()%5==0)
//			{
//				vs_post_text("No. %d; Box center: X=%d; Y=%d\n",vs_loop_counter_get(),centerX,centerY);
//				vs_post_text("Box pos { %d %d %d %d}\n",aabb_coords[0],aabb_coords[1],aabb_coords[2],aabb_coords[3]);
//			}
//		}

        // blink the blue LED to show whether the frame loop is running
        if(vs_loop_counter_get()%50 == 0){
        	vs_led_toggle(VS_LED_1);
        }
        // increase loop counter by 1
        vs_loop_counter_inc();
    }

	return 0;
}

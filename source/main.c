#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"
#include "fsl_pwm.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "hbridge.h"
#include "pixy.h"
#include "fsl_common.h"
#include "Config.h"
#include "servo.h"
#include <math.h>

#define MAX_VECTORS 10

// Checks if there are two perpendicular vectors
bool isIntersection(size_t num_vectors,  uint16_t vectors[]){
	for (size_t i = 0; i < num_vectors; i++) {
		uint16_t x0 = vectors[4*i + 0];
		uint16_t y0 = vectors[4*i + 1];
		uint16_t x1 = vectors[4*i + 2];
		uint16_t y1 = vectors[4*i + 3];
		double m = ((double)x0-(double)x1) / ((double)y0-(double)y1);
		for (size_t j = i+1; j < num_vectors; j++) {
			uint16_t x0_2 = vectors[4*j + 0];
			uint16_t y0_2 = vectors[4*j + 1];
			uint16_t x1_2 = vectors[4*j + 2];
			uint16_t y1_2 = vectors[4*j + 3];
			double m2 = ((double)x0_2-(double)x1_2) / ((double)y0_2-(double)y1_2);
			double p = m * m2;
			if(p <=-0.7 && p>= -1.5){
				/// the vectors are perpendicular
				return true;
			}
		}

	}
}

void differentialSpeed(double angle) {
	if (angle > 10 || angle < -10) {
		HbridgeSpeed(&g_hbridge, SPEED_RIGHT - 10, SPEED_LEFT + 10);
		if (angle > 20) {
			PRINTF("NU\n");
			HbridgeSpeed(&g_hbridge, SPEED_TURN_RIGHT * 1.8, SPEED_TURN_LEFT / 1.5);
		} else if (angle < -20) {
			PRINTF("DA\n");
			HbridgeSpeed(&g_hbridge, SPEED_TURN_RIGHT / 1.7, SPEED_TURN_LEFT * 1.9);
		}
	} else {
		HbridgeSpeed(&g_hbridge, SPEED_RIGHT, SPEED_LEFT);
	}
}



int main(void)
{
    uint16_t vectors[MAX_VECTORS * 4];
    size_t   num_vectors;

    BOARD_InitHardware();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

    HbridgeInit(&g_hbridge,
                CTIMER0_PERIPHERAL,
                CTIMER0_PWM_PERIOD_CH,
                CTIMER0_PWM_1_CHANNEL,
                CTIMER0_PWM_2_CHANNEL,
                GPIO0, 24U,
                GPIO0, 27U);
	HbridgeSpeed(&g_hbridge, SPEED_RIGHT, SPEED_LEFT);

    pixy_t cam1;
    pixy_init(&cam1, LPI2C2, 0x54U, &LP_FLEXCOMM2_RX_Handle, &LP_FLEXCOMM2_TX_Handle);
    pixy_set_led(&cam1, 255, 0, 0);

   volatile double steer = 0;
    while (1)
    {
    	if (pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors) == kStatus_Success) {
    	        double angle = 0.0;
    	        double total_lenght = 0.0;
    	        for (size_t i = 0; i < num_vectors; i++) {
    	            uint16_t x0 = vectors[4*i + 0];
    	            uint16_t y0 = vectors[4*i + 1];
    	            uint16_t x1 = vectors[4*i + 2];
    	            uint16_t y1 = vectors[4*i + 3];
    	            PRINTF("  [%2u] (%u,%u)->(%u,%u)\r\n", (unsigned)i, x0, y0, x1, y1);
    	            double length = sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
					double m = ((double)x0-(double)x1) / ((double)y0-(double)y1);
					angle += m * length;
					total_lenght += length;

				}
				//PRINTF("Total_length = %.2f\n", total_lenght);
				if (num_vectors && total_lenght) {
					angle /= total_lenght;
				}
				if (num_vectors == 0)
						angle = 0.0;
//				if (num_vector > 1)
//					isIntersection(num_vectors, vectors)
				angle *= -1;
				PRINTF("Angle: %u\n" , angle);

    	        if(angle > 0) {
    	        	angle *= STEERING_P_RIGHT * 1.1;
    	        }
    	        else{
					angle *= STEERING_P_LEFT;
    	        }
    	        if (angle > STEERING_LIMIT_RIGHT){
    	        	angle = STEERING_LIMIT_RIGHT;
    	        }
    	        if (angle < STEERING_LIMIT_LEFT){
					angle = STEERING_LIMIT_LEFT;
				}
    	        if(num_vectors !=0)
    	        	Steer(angle + STEERING_OFFSET);

    	        differentialSpeed(angle);

    	    }
		//HbridgeSpeed(&g_hbridge, SPEED_RIGHT, SPEED_LEFT);
    }
}

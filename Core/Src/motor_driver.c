#include "motor_driver.h"

void drive(motor_driver_t* motor, int32_t duty) {

       if (duty > 0) { ///forward

             duty = 4799 - ((duty*4799) / 128) ;

             __HAL_TIM_SET_COMPARE(motor->htim, motor->chan1, 4799); //only works with positive duty
             __HAL_TIM_SET_COMPARE(motor->htim, motor->chan2, duty); //only works with positive duty
       }

       if (duty < 0) { //reverse

             duty = -duty;
             duty = 4799 - ((duty*4799) / 128) ;

             __HAL_TIM_SET_COMPARE(motor->htim, motor->chan1, duty); //only works with positive duty
             __HAL_TIM_SET_COMPARE(motor->htim, motor->chan2, 4799); //only works with positive duty

             // alternate pwm signal between reverse and brake instead of reverse and stop

       }
}

void enable(motor_driver_t* motor){

         HAL_TIM_PWM_Start(motor->htim, motor->chan1); // & -> pointer to variable, initialize PWM channel 1 timer 2
         HAL_TIM_PWM_Start(motor->htim, motor->chan2);
}

void disable(motor_driver_t* motor){

//       _HAL_TIM_SET_COMPARE(motor->htim, motor->chan1, 0);
//       _HAL_TIM_SET_COMPARE(motor->htim, motor->chan2, 0);
       HAL_TIM_PWM_Stop(motor -> htim, motor->chan1);
       HAL_TIM_PWM_Stop(motor -> htim, motor->chan2);
}


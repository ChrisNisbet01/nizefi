#include "trigger_input.h"
#include "trigger_wheel_36_1.h"
#include "queue.h"
#include "rpm_calculator.h"
#include "leds.h"

#include <stddef.h>
#include <stdbool.h>

#define NUM_TEETH 35
#define NUM_MISSING_TEETH 1
#define TOTAL_TEETH (NUM_TEETH + NUM_MISSING_TEETH)

typedef enum trigger_wheel_state_t
{
    trigger_wheel_state_not_synched,
    trigger_wheel_state_synched
} trigger_wheel_state_t;

typedef void (*trigger_36_1_state_handler)(trigger_wheel_36_1_context_st * const context, 
                                           uint32_t const timestamp);

typedef void (*event_callback)(float const tooth_angle_btdc, void * const arg);
typedef struct event_st event_st;

struct event_st
{
    event_callback user_callback; /* user callback*/
    void * user_arg;
};

typedef struct tooth_context_st
{
    CIRCLEQ_ENTRY(tooth_context_st) entry; 

    uint32_t dummy; /* FIXME - Without this, the device crashes. Alignment thing? Buffer overrun? Memory stomp? */

    uint32_t last_timestamp;
    uint32_t last_delta; /* Difference in timestamp between this tooth and the previous timestamp.
                            Valid only if pulse_counter > 1.
                          */
    float crank_angle; /* Retain this information for each tooth. It may vary depending on the configuration of the tooth #1 offset.*/
    float engine_cycle_angle; /* Retain this information for each tooth. It may vary depending on the configuration of the tooth #1 offset.*/

    event_st event;

} tooth_context_st;

struct trigger_wheel_36_1_context_st
{
    trigger_wheel_state_t state;
    trigger_36_1_state_handler crank_trigger_state_hander;
    trigger_36_1_state_handler cam_trigger_state_hander; 

    uint32_t pulse_counter;
    uint32_t revolution_counter;

    tooth_context_st tooth_contexts[NUM_TEETH];

    tooth_context_st * tooth_1; /* When synched, this point to the entry for tooth #1, 
                                    which is the tooth after the missing tooth. 
                                 */
    tooth_context_st * tooth_next;

    rpm_calculator_st * rpm_calculator;
    float rpm;

    bool had_cam_signal; /* Set after receiving a camshaft signal. Indicates next tooth #1 is the start of the 720 degree cycle. */
    bool second_revolution; /* In second half of the 720 degree cycle. */

    float tooth_1_crank_angle; /* -ve indicates BTDC, +ve indicates after TDC. */
    unsigned int tooth_number;
    unsigned int max_tooth_number;
    float crank_angle;
    float engine_cycle_angle;

    CIRCLEQ_HEAD(,tooth_context_st)teeth_queue;

};

static void crank_trigger_wheel_state_not_synched_handler(trigger_wheel_36_1_context_st * const context, 
                                                    uint32_t const timestamp);
static void cam_trigger_wheel_state_not_synched_handler(trigger_wheel_36_1_context_st * const context,
                                                    uint32_t const timestamp);
static void crank_trigger_wheel_state_synched_handler(trigger_wheel_36_1_context_st * const context,
                                                uint32_t const timestamp);
static void cam_trigger_wheel_state_synched_handler(trigger_wheel_36_1_context_st * const context,
                                                uint32_t const timestamp);

static trigger_wheel_36_1_context_st trigger_wheel_context;
static float const degrees_per_tooth = 360.0 / TOTAL_TEETH;
static unsigned int crank_angles[NUM_TEETH + 1] =
{
    0, 10, 20, 30, 40, 50, 60, 70, 80, 90,
    100, 110, 120, 130, 140, 150, 160, 170, 180,
    190, 200, 210, 220, 230, 240, 250, 260, 270,
    280, 290, 300, 310, 320, 330, 340, 0
};

static void set_unsynched(trigger_wheel_36_1_context_st * const context)
{
    context->state = trigger_wheel_state_not_synched;
    context->crank_trigger_state_hander = crank_trigger_wheel_state_not_synched_handler;
    context->cam_trigger_state_hander = cam_trigger_wheel_state_not_synched_handler;

    context->pulse_counter = 0;
    context->tooth_1 = NULL;
}

static void set_synched(trigger_wheel_36_1_context_st * const context)
{
    context->state = trigger_wheel_state_synched;
    context->crank_trigger_state_hander = crank_trigger_wheel_state_synched_handler;
    context->cam_trigger_state_hander = cam_trigger_wheel_state_synched_handler;
    context->had_cam_signal = false; /* Still need a cam signal to know which half of the cycle the engine is in.
                                      */
}

static void crank_trigger_wheel_state_not_synched_handler(trigger_wheel_36_1_context_st * const context, 
                                                          uint32_t const timestamp)
{
    tooth_context_st * current_tooth = context->tooth_next;

    /* TODO: Filter out bogus pulses. */
    context->pulse_counter++;
    current_tooth->last_timestamp = timestamp;

    if (context->pulse_counter == 1)
    {
    }
    else if (context->pulse_counter == 2)
    {
        tooth_context_st * previous_tooth = CIRCLEQ_PREV(current_tooth, entry);

        current_tooth->last_delta = timestamp - previous_tooth->last_timestamp;
        /* Not much else that can be done until another trigger signal 
         * comes in. 
         */
    }
    else
    {
        tooth_context_st * previous_tooth = CIRCLEQ_PREV(current_tooth, entry);
        tooth_context_st * previous_prevous_tooth = CIRCLEQ_PREV(current_tooth, entry);

        current_tooth->last_delta = timestamp - previous_tooth->last_timestamp;
        if ((current_tooth->last_delta < ((previous_tooth->last_delta * 7) / 10)))
        {
            /* Just had a pulse with much shorter delta than the previous. 
             * This is probably tooth #2, which would make the previous 
             * tooth #1.
             */
            context->tooth_1 = previous_tooth;
            context->tooth_number = 2;
            set_synched(context);
        }
        else if ((current_tooth->last_delta < ((previous_tooth->last_delta * 18) / 10))
                  && (current_tooth->last_delta > ((previous_tooth->last_delta * 7) / 10)))
        {
            /* Time between signals is similar to the previous one. Still 
               waiting for skip tooth. */
        }
        else if ((current_tooth->last_delta > ((previous_tooth->last_delta * 18) / 10))
                 && (current_tooth->last_delta < ((previous_tooth->last_delta * 25) / 10)))
        {
            /* Time between signals is about what we expect for a skip 
             * tooth. That would make this tooth #1.
             */
            context->tooth_1 = current_tooth;
            context->tooth_number = 1;
            set_synched(context); 
        }
    }

    context->tooth_next = CIRCLEQ_NEXT(current_tooth, entry);
}

static void cam_trigger_wheel_state_not_synched_handler(trigger_wheel_36_1_context_st * const context,
                                                        uint32_t const timestamp)
{
    context->had_cam_signal = true;
}

static float normalise_angle(float const angle_in, float const maximum_angle)
{
    float angle = angle_in;

    if (angle < 0.0)
    {
        angle += 360.0;
    }
    else if (angle >= maximum_angle)
    {
        angle -= 360.0;
    }

    return angle;
}

static float normalise_crank_angle(float const crank_angle)
{
    return normalise_angle(crank_angle, 360.0);
}

static void update_engine_angles(trigger_wheel_36_1_context_st * const context)
{
    tooth_context_st * current_tooth = context->tooth_next; 
    float angle;
    unsigned int tooth_index = context->tooth_number - 1;

    if (tooth_index >= NUM_TEETH + 1)
    {
        GPIO_ToggleBits(GPIOD, BLUE_LED);
    }

    /* Work out crank angle. */
    angle = normalise_crank_angle(crank_angles[tooth_index] + context->tooth_1_crank_angle);

    context->crank_angle = angle;
    //current_tooth->crank_angle = angle;

    if (context->second_revolution)
    {
        angle += 360.0;
    }
    
    context->engine_cycle_angle = angle;
    //current_tooth->engine_cycle_angle = angle;
}

static void crank_trigger_wheel_state_synched_handler(trigger_wheel_36_1_context_st * const context,
                                                      uint32_t const timestamp)
{
    tooth_context_st * current_tooth = context->tooth_next; 
    tooth_context_st * previous_tooth = CIRCLEQ_PREV(current_tooth, entry);
    uint32_t const last_revolution_timestamp = current_tooth->last_timestamp;

    context->pulse_counter++;
    context->tooth_number++;
    if (context->tooth_number > context->max_tooth_number)
    {
        context->max_tooth_number = context->tooth_number;
    }
    current_tooth->last_timestamp = timestamp;
    current_tooth->last_delta = timestamp - previous_tooth->last_timestamp;

    update_engine_angles(context);

    /* It is expected that the cam signal arrives some time just 
     * before tooth #1. 
     */
    /* TODO: Validate time deltas between teeth. 
     */
    if (current_tooth == context->tooth_1)
    {
        uint32_t rotation_time_32 = timestamp - last_revolution_timestamp;
        float const rotation_time = (float)rotation_time_32 / 1000000;

        /* Just found tooth #1. */

        context->tooth_number = 1;
        context->second_revolution = !context->had_cam_signal;
        context->had_cam_signal = false;

        if (!context->second_revolution)
        {
            debug_injector_pulse();
        }
        /* TODO: update rmp based upon current RPM. If RPM high, 
         * update less often, so that the time between updates remains 
         * more constant and doesn't increase CPU load with RPM so much. 
         * So maybe once/rev above 3000rpm, twice/rev between 1000 and 
         * 3000, and  four times/rev below 1000 rpm. 
         */
        rpm_calculator_update(context->rpm_calculator, rotation_time, 1.0);
        context->revolution_counter++;
    }

    context->tooth_next = CIRCLEQ_NEXT(current_tooth, entry);
}

static void cam_trigger_wheel_state_synched_handler(trigger_wheel_36_1_context_st * const context,
                                                    uint32_t const timestamp)
{
    context->had_cam_signal = true; 
}

trigger_wheel_36_1_context_st * trigger_36_1_init(void)
{
    trigger_wheel_36_1_context_st * context = &trigger_wheel_context;
    size_t index;

    context->tooth_1_crank_angle = 0.0; /* TODO: Configurable. */
    context->max_tooth_number = 0;
    set_unsynched(context);

    context->revolution_counter = 0;

    CIRCLEQ_INIT(&context->teeth_queue);
    for (index = 0; index < NUM_TEETH; index++)
    {
        tooth_context_st * const tooth_context = &context->tooth_contexts[index];

        CIRCLEQ_INSERT_TAIL(&context->teeth_queue, tooth_context, entry);
    }
    context->tooth_next = CIRCLEQ_FIRST(&context->teeth_queue);

    /* Until synchronised, the entry representing tooth #1 is 
     * unknown. 
     */

    context->rpm_calculator = rpm_calculator_get();
    rpm_calculator_smoothing_factor_set(context->rpm_calculator, 0.98);
    context->rpm = 0.0;

    return context;
}

void trigger_36_1_handle_crank_pulse(trigger_wheel_36_1_context_st * const context,
                               uint32_t const timestamp)
{
    context->crank_trigger_state_hander(context, timestamp);
}

void trigger_36_1_handle_cam_pulse(trigger_wheel_36_1_context_st * const context,
                                     uint32_t const timestamp)
{
    context->cam_trigger_state_hander(context, timestamp);
}

float trigger_36_1_rpm_get(trigger_wheel_36_1_context_st * const context)
{
    return rpm_calculator_smoothed_rpm_get(context->rpm_calculator);
}

float trigger_36_1_crank_angle_get(trigger_wheel_36_1_context_st * const context)
{
    tooth_context_st * current_tooth = context->tooth_next;
    tooth_context_st * previous_tooth = CIRCLEQ_PREV(current_tooth, entry); 
    //float crank_angle = previous_tooth->crank_angle;
    float crank_angle = context->crank_angle; 

    /* TODO: Account for time elapsed since this tooth was 
     * encountered. 
     */

    return crank_angle;
}

float trigger_36_1_engine_cycle_angle_get(trigger_wheel_36_1_context_st * const context)
{
    tooth_context_st * current_tooth = context->tooth_next;
    tooth_context_st * previous_tooth = CIRCLEQ_PREV(current_tooth, entry);
    //float engine_cycle_angle = previous_tooth->crank_angle;
    float engine_cycle_angle = context->engine_cycle_angle; 

    /* TODO: Account for time elapsed since this tooth was 
     * encountered. 
     */

    //return engine_cycle_angle;
    return context->max_tooth_number;
}

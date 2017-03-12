#include "trigger_input.h"
#include "trigger_wheel_36_1.h"
#include "queue.h"
#include "rpm_calculator.h"
#include "leds.h"

#include <coocox.h>

#include <stddef.h>
#include <stdbool.h>
#include <inttypes.h>

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

typedef struct event_st event_st;

struct event_st
{
    trigger_event_callback user_callback; /* user callback */
    void * user_arg;
};

typedef struct tooth_context_st
{
    CIRCLEQ_ENTRY(tooth_context_st) entry; 

    uint32_t last_timestamp;
    uint32_t last_delta; /* Difference in timestamp between this tooth and the previous timestamp.
                            Valid only if pulse_counter > 1.
                          */
    float crank_angle; /* Retain this information for each tooth. It may vary depending on the configuration of the tooth #1 offset.*/
    int index; 

    event_st event[2]; /* One callback allowed for each half of the engine cycle. */

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
    unsigned int tooth_number; /* Note - Starts at 1.*/

    rpm_calculator_st * rpm_calculator;

    bool had_cam_signal; /* Set after receiving a camshaft signal. Indicates next tooth #1 is the start of the 720 degree cycle. */
    bool second_revolution; /* In second half of the 720 degree cycle. */

    float tooth_1_crank_angle; /* -ve indicates BTDC, +ve indicates after TDC. */
    uint32_t timestamp; /* timestamp taken when the lat tooth was processed. */
    float crank_angle;
    float engine_cycle_angle;

    CIRCLEQ_HEAD(,tooth_context_st) teeth_queue;

    OS_MutexID teeth_mutex;
};

static void crank_trigger_wheel_state_not_synched_handler(trigger_wheel_36_1_context_st * const context, 
                                                          uint32_t const timestamp);
static void crank_trigger_wheel_state_synched_handler(trigger_wheel_36_1_context_st * const context,
                                                      uint32_t const timestamp);
static void cam_trigger_wheel_state_handler(trigger_wheel_36_1_context_st * const context,
                                            uint32_t const timestamp);

static trigger_wheel_36_1_context_st trigger_wheel_context;
static float const degrees_per_tooth = 360.0 / TOTAL_TEETH;

static unsigned int const crank_angles[NUM_TEETH] =
{
    0, 10, 20, 30, 40, 50, 60, 70, 80, 90,
    100, 110, 120, 130, 140, 150, 160, 170, 180,
    190, 200, 210, 220, 230, 240, 250, 260, 270,
    280, 290, 300, 310, 320, 330, 340
};

static event_st event_callbacks[2][NUM_TEETH];

/* Cripes. This CIRCLEQ implementation will return a pointer to 
 * the list head, not just entries in the list. 
 */
tooth_context_st * previous_tooth_get(trigger_wheel_36_1_context_st * const context, tooth_context_st * const tooth)
{
    tooth_context_st * previous;

    previous = CIRCLEQ_PREV(tooth, entry);
    if (previous == (void *)&context->teeth_queue)
    {
        previous = CIRCLEQ_PREV(previous, entry);
    }

    return previous;
}

tooth_context_st * next_tooth_get(trigger_wheel_36_1_context_st * const context, tooth_context_st * const tooth)
{
    tooth_context_st * next;

    next = CIRCLEQ_NEXT(tooth, entry);
    if (next == (void *)&context->teeth_queue)
    {
        next = CIRCLEQ_NEXT(next, entry);
    }

    return next;
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

/* XXX Only needs to be done once when the trigger wheel is 
 * synched. 
 * The engine_cycle angle can be determined when needed by 
 * adding 360 degrees if in the second cycle. Beware of race 
 * when doing this. 
 */
static void update_engine_angles(trigger_wheel_36_1_context_st * const context)
{
    size_t index;
    tooth_context_st * tooth = context->tooth_1; 

    for (index = 0; index < NUM_TEETH; index++, tooth = next_tooth_get(context, tooth));
    {
        /* Work out crank angle. */
        tooth->crank_angle = 11.0 + index + normalise_crank_angle((float)crank_angles[index] + context->tooth_1_crank_angle);
    }
}

static void set_unsynched(trigger_wheel_36_1_context_st * const context)
{
    context->state = trigger_wheel_state_not_synched;
    context->crank_trigger_state_hander = crank_trigger_wheel_state_not_synched_handler;
    context->cam_trigger_state_hander = cam_trigger_wheel_state_handler; 
    context->pulse_counter = 0;
    context->tooth_1 = NULL;
}

static void set_synched(trigger_wheel_36_1_context_st * const context)
{
    context->state = trigger_wheel_state_synched;
    context->crank_trigger_state_hander = crank_trigger_wheel_state_synched_handler;
    context->had_cam_signal = false; /* Still need a cam signal to know which half of the cycle the engine is in.
                                      */
    update_engine_angles(context);
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
        tooth_context_st * previous_tooth = previous_tooth_get(context, current_tooth);

        current_tooth->last_delta = timestamp - previous_tooth->last_timestamp;
        /* Not much else that can be done until another trigger signal 
         * comes in. 
         */
    }
    else
    {
        tooth_context_st * previous_tooth = previous_tooth_get(context, current_tooth);
        tooth_context_st * previous_prevous_tooth = previous_tooth_get(context, previous_tooth);

        current_tooth->last_delta = timestamp - previous_tooth->last_timestamp;
        if ((current_tooth->last_delta < ((previous_tooth->last_delta * 7) / 10)))
        {
            /* Just had a pulse with much shorter delta than the previous. 
             * This is probably tooth #2, which would make the previous 
             * tooth #1.
             */
            context->tooth_1 = previous_tooth;
            context->tooth_number = 2;
            context->timestamp = timestamp;
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
            context->tooth_1 = current_tooth; /* Set to previous tooth because the next tooth is */
            context->tooth_number = 1;
            context->timestamp = timestamp;
            set_synched(context);
        }
    }

    context->tooth_next = next_tooth_get(context, current_tooth);

    return;
}

static void cam_trigger_wheel_state_handler(trigger_wheel_36_1_context_st * const context,
                                            uint32_t const timestamp)
{
    context->had_cam_signal = true;
}

static void execute_callbacks(bool const second_revolution, 
                              unsigned int const tooth_number, 
                              uint32_t const timestamp)
{
    unsigned int const event_index = tooth_number - 1;

    /* timestamp is the time at which the tooth was encountered. 
     */
    if (event_callbacks[second_revolution][event_index].user_callback != NULL)
    {
        event_callbacks[second_revolution][event_index].user_callback(crank_angles[event_index],
                                                                      timestamp,
                                                                      event_callbacks[second_revolution][event_index].user_arg);
    }
}

static void crank_trigger_wheel_state_synched_handler(trigger_wheel_36_1_context_st * const context,
                                                      uint32_t const timestamp)
{
    tooth_context_st * current_tooth = context->tooth_next; 
    tooth_context_st * previous_tooth = previous_tooth_get(context, current_tooth);
    uint32_t const last_revolution_timestamp = current_tooth->last_timestamp;
    unsigned int tooth_number;

    context->pulse_counter++;
    current_tooth->last_timestamp = timestamp;
    current_tooth->last_delta = timestamp - previous_tooth->last_timestamp;

    /* It is expected that the cam signal arrives some time just 
     * before tooth #1 and before the start of the first half of the
     * engine cycle. 
     */

    CoEnterMutexSection(context->teeth_mutex);

    context->timestamp = timestamp;
    tooth_number = context->tooth_number + 1;
    if (tooth_number == NUM_TEETH + 1)
    {
        tooth_number = 1;
        context->second_revolution = !context->had_cam_signal;
        context->had_cam_signal = false;
        context->revolution_counter++;
    }
    context->tooth_number = tooth_number;
    context->tooth_next = next_tooth_get(context, current_tooth); 

    CoLeaveMutexSection(context->teeth_mutex); 

    execute_callbacks(context->second_revolution, tooth_number, timestamp);

    /* TODO: Validate time deltas between teeth. 
     */
    if (context->tooth_number == 1)
    {
        uint32_t const rotation_time_us = timestamp - last_revolution_timestamp;
        float const rotation_time_seconds = (float)rotation_time_us / 1000000;

        /* TODO: call rpm_calculator_update based upon current RPM. If 
         * RPM high, update less often so that the time between updates 
         * remains more constant and doesn't increase CPU load with RPM 
         * so much. So maybe once/rev above 3000rpm, twice/rev between 
         * 1000 and 3000, and  four times/rev below 1000 rpm. 
         */
        rpm_calculator_update(context->rpm_calculator, 360.0, rotation_time_seconds);
    }
}

trigger_wheel_36_1_context_st * trigger_36_1_init(void)
{
    trigger_wheel_36_1_context_st * context = &trigger_wheel_context;
    size_t index;

    context->tooth_1_crank_angle = 0.0; /* TODO: Configurable. */

    set_unsynched(context);
    context->teeth_mutex = CoCreateMutex();

    context->revolution_counter = 0;

    CIRCLEQ_INIT(&context->teeth_queue);
    for (index = 0; index < NUM_TEETH; index++)
    {
        tooth_context_st * const tooth_context = &context->tooth_contexts[index];

        tooth_context->index = index;
        CIRCLEQ_INSERT_TAIL(&context->teeth_queue, tooth_context, entry);
    }
    context->tooth_next = CIRCLEQ_FIRST(&context->teeth_queue);

    /* Until synchronised, the entry representing tooth #1 is 
     * unknown. 
     */

    context->rpm_calculator = rpm_calculator_get();
    rpm_calculator_smoothing_factor_set(context->rpm_calculator, 0.98); /* TODO: Make configurable. */

    return context;
}

void trigger_36_1_register_callback(trigger_wheel_36_1_context_st * const context,
                                    float const engine_cycle_angle,
                                    trigger_event_callback callback,
                                    void * const user_arg)
{
    size_t index;
    unsigned int cycle;

    /* FIXME - This isn't very clean. */
    for (cycle = 0; cycle < 2; cycle++)
    {
        for (index = 0; index < NUM_TEETH; index++)
        {
            if (((float)crank_angles[index] + (360.0 * cycle)) == engine_cycle_angle)
            {
                event_callbacks[cycle][index].user_callback = callback;
                event_callbacks[cycle][index].user_arg = user_arg;
                goto done;
            }
        }
    }

done:
    return;
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

float trigger_36_1_angle_get(trigger_wheel_36_1_context_st * const context, bool const engine_angle)
{
    unsigned int tooth_number;
    float crank_angle;
    uint32_t us_since_tooth_passed;
    uint32_t tooth_timestamp;
    uint32_t time_now;
    float degrees_since_tooth_passed;
    bool previous_tooth_in_second_revolution;

    CoEnterMutexSection(context->teeth_mutex);

    tooth_timestamp = context->timestamp;
    tooth_number = context->tooth_number;
    previous_tooth_in_second_revolution = (context->second_revolution && context->tooth_number != 1)
        || (!context->second_revolution && context->tooth_number == 1); 

    CoLeaveMutexSection(context->teeth_mutex);

    time_now = hi_res_counter_val();
    us_since_tooth_passed = time_now - tooth_timestamp;
    degrees_since_tooth_passed = rpm_calcuator_get_degrees_turned(context->rpm_calculator, (float)us_since_tooth_passed / 1000000.0);
    crank_angle = normalise_crank_angle((float)crank_angles[tooth_number - 1] + context->tooth_1_crank_angle + degrees_since_tooth_passed);

    if (engine_angle && previous_tooth_in_second_revolution)
    {
        crank_angle += 360.0;
    }

    return crank_angle;
}

float trigger_36_1_crank_angle_get(trigger_wheel_36_1_context_st * const context)
{
    return trigger_36_1_angle_get(context, false);
}

float trigger_36_1_engine_cycle_angle_get(trigger_wheel_36_1_context_st * const context)
{
    return trigger_36_1_angle_get(context, true);
}


#include "trigger_input.h"
#include "trigger_wheel_36_1.h"
#include "queue.h"
#include "rpm_calculator.h"
#include "leds.h"
#include "hi_res_timer.h"
#include "utils.h"

#include <coocox.h>

#include <stddef.h>
#include <stdbool.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>

#define NUM_TEETH 35
#define NUM_MISSING_TEETH 1
#define TOTAL_TEETH (NUM_TEETH + NUM_MISSING_TEETH)
#define NUM_EVENT_ENTRIES 20 /* Ensure is enough to cover all injectors + ignition outputs 
                                and anything else that requires updating at a particualr engine angle. */

typedef void (* trigger_36_1_state_handler)(trigger_wheel_36_1_context_st * const context, 
                                            uint32_t const timestamp);

typedef float (* trigger_36_1_angle_get_handler)(trigger_wheel_36_1_context_st * const context, bool const engine_angle);
typedef float (* trigger_36_1_rotation_time_get_handler)(trigger_wheel_36_1_context_st * const context, float const rotation_angle);

typedef struct event_st event_st;

struct event_st
{
    SLIST_ENTRY(event_st) entry;
    trigger_event_callback user_callback; /* user callback */
    void * user_arg;
};

typedef SLIST_HEAD(event_list_head_st, event_st) event_list_head_st;

typedef struct tooth_context_st
{
    CIRCLEQ_ENTRY(tooth_context_st) entry; 

    uint32_t timestamp;
    uint32_t time_since_previous_tooth; /* Difference in timestamp between this tooth and the previous timestamp.
                                            Valid only if pulse_counter > 1.
                                        */
    float crank_angle; /* Retain this information for each tooth. It may vary depending on the configuration of the tooth #1 offset.*/

} tooth_context_st;

struct trigger_wheel_36_1_context_st
{
    /* Trigger input crank and cam state handlers are called by the 
     * trigger_input task. 
     */
    trigger_36_1_state_handler crank_trigger_state_handler;
    trigger_36_1_state_handler cam_trigger_state_handler;
    trigger_36_1_angle_get_handler angle_get_handler;
    trigger_36_1_rotation_time_get_handler rotation_time_get_handler;

    uint32_t pulse_counter;
    uint32_t revolution_counter;

    tooth_context_st tooth_contexts[NUM_TEETH];

    /* Note that the event callbacks are in tooth order, starting 
     * at tooth #1. 
     */
    event_list_head_st event_callbacks[2][NUM_TEETH];

    tooth_context_st * tooth_1; /* When synched, this points to the entry for tooth #1, 
                                    which is the tooth after the missing tooth. 
                                 */
    tooth_context_st * tooth_next;
    unsigned int tooth_number; /* Note - Starts at 1.*/

    rpm_calculator_st * rpm_calculator;

    bool had_cam_signal; /* Set after receiving a camshaft signal. Indicates next tooth #1 is the start of the 720 degree cycle. */

    /* TODO - Support two-stroke and batch fire configurations 
     * where the engine cycle is only 360 degrees. 
     */
    bool second_revolution; /* In second half of the 720 degree cycle. */

    float tooth_1_crank_angle; /* -ve indicates BTDC, +ve indicates ATDC. */
    uint32_t timestamp; /* timestamp taken when the last tooth was processed. */

    CIRCLEQ_HEAD(,tooth_context_st) teeth_queue;

    OS_MutexID teeth_mutex;
};

typedef void (* event_list_callback_fn)(event_st const * const event, void * const user_arg);

static void crank_trigger_wheel_state_not_synched_handler(trigger_wheel_36_1_context_st * const context, 
                                                          uint32_t const timestamp);
static void crank_trigger_wheel_state_synched_handler(trigger_wheel_36_1_context_st * const context,
                                                      uint32_t const timestamp);
static void cam_trigger_wheel_state_handler(trigger_wheel_36_1_context_st * const context,
                                            uint32_t const timestamp);

static float rpm_smoothing_factor = 0.98; /* TODO - Make configurable. */
static float tooth_1_crank_angle = 0.0; /* TODO - Make configurable. */

static event_list_head_st free_event_entries;
static event_st event_entries[NUM_EVENT_ENTRIES];

static trigger_wheel_36_1_context_st trigger_wheel_context;

static unsigned int const trigger_wheel_tooth_angles[NUM_TEETH] =
{
    0, 10, 20, 30, 40, 50, 60, 70, 80, 90,
    100, 110, 120, 130, 140, 150, 160, 170, 180,
    190, 200, 210, 220, 230, 240, 250, 260, 270,
    280, 290, 300, 310, 320, 330, 340
};

/* Crank angles at each tooth after the tooth #1 crank angle 
   offset has been applied. */
static float runtime_crank_angles[NUM_TEETH + 1] = /* +1 is to avoid stupid array-bounds compiler warning */
{
    0, 10, 20, 30, 40, 50, 60, 70, 80, 90,
    100, 110, 120, 130, 140, 150, 160, 170, 180,
    190, 200, 210, 220, 230, 240, 250, 260, 270,
    280, 290, 300, 310, 320, 330, 340
};

float rpm_smoothing_factor_get(void)
{
    /* TODO - Make configurable. */
    return rpm_smoothing_factor;
}

float tooth_1_crank_angle_get(void)
{
    /* TODO - Make configurable. */
    return tooth_1_crank_angle;
}

/* All event_entry list code relating to accessing of the list 
 * kept together. This should make it easier to use a different 
 * list type if the need should arise. 
 */
static inline void event_entry_list_init(event_list_head_st * const head)
{
    SLIST_INIT(head);
}

static inline void event_entry_list_insert(event_list_head_st * head, event_st * const event)
{
    SLIST_INSERT_HEAD(head, event, entry);
}

static void event_entry_init(void)
{
    size_t index;

    event_entry_list_init(&free_event_entries);

    /* Add all events into a linked list for easy retrieval. Easier 
     * to keep track of which ones are free if support for 
     * deregistering callbacks is ever supported. 
    */
    for (index = 0; index < NUM_EVENT_ENTRIES; index++)
    {
        SLIST_INSERT_HEAD(&free_event_entries, &event_entries[index], entry);
    }
}

static event_st * event_entry_get(void)
{
    event_st * event = SLIST_FIRST(&free_event_entries);

    if (event != NULL)
    {
        SLIST_REMOVE_HEAD(&free_event_entries, entry);
    }

    return event;
}

static void iterate_events(event_list_head_st * const head, event_list_callback_fn callback, void * const user_arg)
{
    event_st * event;

    SLIST_FOREACH(event, head, entry)
    {
        callback(event, user_arg);
    }
}

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

/* XXX Only needs to be done once when the trigger wheel is 
 * synched. 
 * The engine_cycle angle can be determined when needed by 
 * adding 360 degrees if in the second cycle. Beware of race 
 * when doing this. 
 */
static void update_engine_angles(trigger_wheel_36_1_context_st * const context)
{
    tooth_context_st * tooth = context->tooth_1; 
    size_t index; 

    for (index = 0; index < NUM_TEETH; index++);
    {
        /* Work out crank angle. */
        runtime_crank_angles[index] = normalise_crank_angle((float)trigger_wheel_tooth_angles[index] + context->tooth_1_crank_angle);
        tooth->crank_angle = runtime_crank_angles[index];

        tooth = next_tooth_get(context, tooth);
    }
}

static float trigger_36_1_unsynched_angle_get(trigger_wheel_36_1_context_st * const context, bool const engine_angle)
{
    /* Until the trigger wheel code is synched in the crank angle 
     * returned is 0.0.
     */
    (void)context;
    (void)engine_angle;

    return 0.0;
}

static float trigger_36_1_synched_angle_get(trigger_wheel_36_1_context_st * const context, bool const engine_angle)
{
    unsigned int tooth_number;
    float crank_angle;
    uint32_t ticks_since_tooth_passed;
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
    ticks_since_tooth_passed = time_now - tooth_timestamp;
    degrees_since_tooth_passed = rpm_calcuator_get_degrees_turned(context->rpm_calculator, (float)ticks_since_tooth_passed / TIMER_FREQUENCY);

    crank_angle = normalise_crank_angle(runtime_crank_angles[tooth_number - 1] + degrees_since_tooth_passed);

    if (engine_angle && previous_tooth_in_second_revolution)
    {
        crank_angle += 360.0;
    }

    return crank_angle;
}

static float trigger_36_1_synched_rotation_time_get(trigger_wheel_36_1_context_st * const context, float const rotation_angle)
{
    return rpm_calcuator_get_time_to_rotate_angle(context->rpm_calculator, rotation_angle);
}

static float trigger_36_1_unsynched_rotation_time_get(trigger_wheel_36_1_context_st * const context, float const rotation_angle)
{
    (void)context;
    (void)rotation_angle;

    return NAN;
}

float trigger_36_1_rotation_time_get(trigger_wheel_36_1_context_st * const context, float const rotation_angle)
{
    return context->rotation_time_get_handler(context, rotation_angle);
}

static void set_unsynched(trigger_wheel_36_1_context_st * const context)
{
    context->crank_trigger_state_handler = crank_trigger_wheel_state_not_synched_handler;
    context->cam_trigger_state_handler = cam_trigger_wheel_state_handler;
    context->angle_get_handler = trigger_36_1_unsynched_angle_get;
    context->rotation_time_get_handler = trigger_36_1_unsynched_rotation_time_get;
    context->pulse_counter = 0;
    context->tooth_1 = NULL;
    rpm_calculator_init(context->rpm_calculator, rpm_smoothing_factor); 
}

static void set_synched(trigger_wheel_36_1_context_st * const context)
{
    context->crank_trigger_state_handler = crank_trigger_wheel_state_synched_handler;
    context->cam_trigger_state_handler = cam_trigger_wheel_state_handler; /* Doesn't change, but for consitency we'll include here. */
    context->angle_get_handler = trigger_36_1_synched_angle_get;
    context->rotation_time_get_handler = trigger_36_1_synched_rotation_time_get;
    context->had_cam_signal = false; /* Still need a cam signal to know which half of the cycle the engine is in.
                                      */
    update_engine_angles(context);
}

static inline uint32_t single_tooth_change_low_limit(uint32_t const time)
{
    return (time * 7) / 10;
}

static inline uint32_t single_tooth_change_high_limit(uint32_t const time)
{
    return (time * 17) / 10;
}

static inline uint32_t skip_tooth_change_high_limit(uint32_t const time)
{
    return (time * 27) / 10;
}


static bool is_second_tooth_after_skip_tooth(uint32_t const this_delta, uint32_t const previous_delta)
{
    return this_delta < single_tooth_change_low_limit(previous_delta);
}

static bool interval_matches_previous_tooth(uint32_t const this_delta, uint32_t const previous_delta)
{
    return (this_delta <= single_tooth_change_high_limit(previous_delta))
    && (this_delta > single_tooth_change_low_limit(previous_delta));
}

static bool interval_matches_skip_tooth(uint32_t const this_delta, uint32_t const previous_delta)
{
    return (this_delta > single_tooth_change_high_limit(previous_delta))
           && (this_delta <= skip_tooth_change_high_limit(previous_delta));
}

static void crank_trigger_wheel_state_not_synched_handler(trigger_wheel_36_1_context_st * const context,
                                                          uint32_t const timestamp)
{
    tooth_context_st * current_tooth = context->tooth_next;

    /* TODO: Filter out bogus pulses. */
    context->pulse_counter++;
    current_tooth->timestamp = timestamp;

    if (context->pulse_counter == 1)
    {
        /* Nothing to do. */
    }
    else if (context->pulse_counter == 2)
    {
        tooth_context_st * previous_tooth = previous_tooth_get(context, current_tooth);
        int32_t time_since_previous_tooth = timestamp - previous_tooth->timestamp; 

        if (time_since_previous_tooth > 0)
        {
            current_tooth->time_since_previous_tooth = timestamp - previous_tooth->timestamp;
        }
        else
        {
            current_tooth->time_since_previous_tooth = 0;
            context->pulse_counter = 1;
        }
        /* Not much else that can be done until another trigger signal 
         * comes in. 
         */
    }
    else
    {
        tooth_context_st * previous_tooth = previous_tooth_get(context, current_tooth);
        int32_t time_since_previous_tooth = timestamp - previous_tooth->timestamp; 

        if (time_since_previous_tooth > 0)
        {
            current_tooth->time_since_previous_tooth = time_since_previous_tooth;

            if (is_second_tooth_after_skip_tooth(current_tooth->time_since_previous_tooth, previous_tooth->time_since_previous_tooth))
            {
                /* Just had a pulse with much shorter delta than the previous. 
                 * This is probably tooth #2, which would make the previous 
                 * tooth #1.
                 */
                context->tooth_1 = previous_tooth;
                context->tooth_number = 2;
                context->timestamp = timestamp;
                context->second_revolution = !context->had_cam_signal;
                set_synched(context);
            }
            else if (interval_matches_previous_tooth(current_tooth->time_since_previous_tooth, previous_tooth->time_since_previous_tooth))
            {
                /* Time between signals is similar to the previous one. Still 
                   waiting for skip tooth. */
            }
            else if (interval_matches_skip_tooth(current_tooth->time_since_previous_tooth, previous_tooth->time_since_previous_tooth))
            {
                /* Time between signals is about what we expect for a skip 
                 * tooth. That would make this tooth #1.
                 */
                context->tooth_1 = current_tooth; /* Set to previous tooth because the next tooth is */
                context->tooth_number = 1;
                context->timestamp = timestamp;
                context->second_revolution = !context->had_cam_signal;
                set_synched(context);
            }
        }
        else
        {
            /* Something bad is happening.*/ 
        }
    }

    context->tooth_next = next_tooth_get(context, current_tooth);

    return;
}

static void cam_trigger_wheel_state_handler(trigger_wheel_36_1_context_st * const context,
                                            uint32_t const timestamp)
{
    (void)timestamp; /* Is it necessary to record when the cam signal came in, or just that we had it. */

    context->had_cam_signal = true;
}

typedef struct event_callback_info_st
{
    float crank_angle;
    uint32_t timestamp;
} event_callback_info_st;

static void event_callback(event_st const * const event, void * const user_arg)
{
    event_callback_info_st * callback_info = user_arg;

    event->user_callback(callback_info->crank_angle,
                         callback_info->timestamp,
                         event->user_arg);
}

static void execute_engine_cycle_events(trigger_wheel_36_1_context_st * const context,
                                        unsigned int const tooth_number, 
                                        uint32_t const timestamp)
{
    unsigned int const event_index = tooth_number - 1;
    event_callback_info_st callback_info;

    callback_info.crank_angle = runtime_crank_angles[event_index];
    callback_info.timestamp = timestamp;

    iterate_events(&context->event_callbacks[context->second_revolution][event_index], 
                   event_callback,
                   &callback_info);

}

static bool validate_tooth_interval(unsigned int tooth_number,
                                    int32_t const this_delta,
                                    int32_t const previous_delta,
                                    uint32_t * const missed_tooth_delta)
{
    bool tooth_interval_is_valid;

    /* If it has been more than 1 second since the last trigger 
     * input we'll go to lost sych state. 
     */
    if (this_delta <= 0)
    {
        printf("-ve %"PRId32"\r\n", this_delta);
        tooth_interval_is_valid = false;
        goto done;
    }

    if (this_delta > TIMER_FREQUENCY)
    {
        printf("1 %"PRIu32"\r\n", this_delta);
        tooth_interval_is_valid = false;
        goto done;
    }

    if (tooth_number == 1)
    {
        if (!is_second_tooth_after_skip_tooth(this_delta, previous_delta))
        {
            printf("2 %"PRIu32" %"PRIu32"\r\n", this_delta, previous_delta);
            tooth_interval_is_valid = false;
            goto done;
        }
    }
    else if (tooth_number == NUM_TEETH)
    {
        if (!interval_matches_skip_tooth(this_delta, previous_delta))
        {
            printf("3 %"PRIu32" %"PRIu32"\r\n", this_delta, previous_delta);
            tooth_interval_is_valid = false;
            goto done;
        }
    }
    else if (!interval_matches_previous_tooth(this_delta, previous_delta))
    {
        printf("4 %"PRIu32" %"PRIu32"\r\n", this_delta, previous_delta);

        /* Maybe if it's just a single missed tooth we can recover 
         * from this by pretending we got the tooth half this delta's 
         * time ago. Need to be careful to check that this is a 
         * one-off/rare thing (maybe once per few thousand signals). 
         */
        if (missed_tooth_delta != NULL && interval_matches_skip_tooth(this_delta, previous_delta))
        {
            *missed_tooth_delta = this_delta / 2;
        }
        tooth_interval_is_valid = false;
        goto done;
    }

    tooth_interval_is_valid = true; 

done:
    return tooth_interval_is_valid;
}

static void crank_trigger_wheel_state_synched_handler(trigger_wheel_36_1_context_st * const context,
                                                      uint32_t const timestamp)
{
    tooth_context_st * current_tooth = context->tooth_next; 
    tooth_context_st * previous_tooth = previous_tooth_get(context, current_tooth);
    uint32_t const last_revolution_timestamp = current_tooth->timestamp;
    unsigned int tooth_number;
    bool lost_synch = false;
    uint32_t missed_tooth_delta = 0;
    int32_t time_since_previous_tooth;

    context->pulse_counter++;
    current_tooth->timestamp = timestamp;
    time_since_previous_tooth = timestamp - previous_tooth->timestamp;

    /* XXX - It appears that some interrupts are missed. Why? Poor 
     * signal quality? Poor code? Too much CPU load? (surely not). 
     * And why are the missed interrupts so regular? i.e. they seem 
     * to arrive at a rate of about 50 per timer rollover, so about 
     * once every 85 seconds. Can almost set your clock by them? Due 
     * to external signal or internal to CPU? 
     * Hmm, I've just worked out that a 32 bit counter increasing at 
     * a rate of 50MHz will roll over every ~85 seconds. 
     * Coincidence? I think not. What's the STM32F3 timer clock rate
     * CPU speed (whatever) I wonder?
     */
    if (!validate_tooth_interval(context->tooth_number,
                                 time_since_previous_tooth,
                                 previous_tooth->time_since_previous_tooth,
                                 &missed_tooth_delta))
    {
        lost_synch = true;
        goto done;
    }

    current_tooth->time_since_previous_tooth = time_since_previous_tooth;

    /* It is expected that the cam signal arrives some time just 
     * before tooth #1 and before the start of the first half of the
     * engine cycle. 
     */

    CoEnterMutexSection(context->teeth_mutex);

    context->timestamp = timestamp;
    tooth_number = context->tooth_number + 1;
    if (tooth_number == NUM_TEETH + 1)
    {
        tooth_number = 1; /* Back to tooth #1 */

        /* Ensure that the cam signal has arrived if not currently in 
         * the first half of the engine cycle, and that it has not 
         * arrived if in the second half. 
         */
        if ((context->second_revolution ^ context->had_cam_signal))
        {
            lost_synch = true;
        }

        context->second_revolution = !context->had_cam_signal;
        context->had_cam_signal = false;
        context->revolution_counter++;
    }
    context->tooth_number = tooth_number;
    context->tooth_next = next_tooth_get(context, current_tooth); 

    CoLeaveMutexSection(context->teeth_mutex); 

    if (lost_synch)
    {
        goto done;
    }

    execute_engine_cycle_events(context, tooth_number, timestamp);

    /* TODO: Validate time deltas between teeth. 
     */
    if (context->tooth_number == 1)
    {
        uint32_t const rotation_time_ticks = timestamp - last_revolution_timestamp;
        float const rotation_time_seconds = (float)rotation_time_ticks / TIMER_FREQUENCY;

        /* TODO: call rpm_calculator_update based upon current RPM. If 
         * RPM high, update less often so that the time between updates 
         * remains more constant and doesn't increase CPU load with RPM 
         * so much. So maybe once/rev above 3000rpm, twice/rev between 
         * 1000 and 3000, and  four times/rev below 1000 rpm. 
         */
        rpm_calculator_update(context->rpm_calculator, 360.0, rotation_time_seconds);
    }

    /* To avoid the check for lost_synch (which is know is false 
     * at this point), just return. Reduces CPU load.
     */
    return;

done:
    if (lost_synch)
    {
        /* TODO: Update statistics. */
        set_unsynched(context);
    }

    return;
}

trigger_wheel_36_1_context_st * trigger_36_1_init(void)
{
    trigger_wheel_36_1_context_st * context = &trigger_wheel_context;
    size_t index;

    /* TODO: Support changing this angle while the engine is 
     * turning. 
     */
    context->tooth_1_crank_angle = tooth_1_crank_angle_get();

    /* RPM calculator is needed before setting unsynched state as 
     * set_unsynched references the calculator. 
     */
    context->rpm_calculator = rpm_calculator_get(rpm_smoothing_factor_get());

    set_unsynched(context);

    context->teeth_mutex = CoCreateMutex();

    context->revolution_counter = 0;

    event_entry_init();
    for (index = 0; index < NUM_TEETH; index++)
    {
        event_entry_list_init(&context->event_callbacks[0][index]);
        event_entry_list_init(&context->event_callbacks[1][index]);
    }

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

    return context;
}

static int find_closest_tooth(float const engine_cycle_angle, int const cycle)
{
    int index;

    for (index = NUM_TEETH - 1; index >= 0; index--)
    {
        if ((runtime_crank_angles[index] + (360.0 * cycle)) <= engine_cycle_angle)
        {
            break;
        }
    }

    return index;
}

void trigger_36_1_register_callback(trigger_wheel_36_1_context_st * const context,
                                    float const engine_cycle_angle,
                                    trigger_event_callback callback,
                                    void * const user_arg)
{
    int cycle;
    int index;

    /* Find the tooth that most closely matches the desired angle. 
     * Note that the desired angle is specified in engine cycle 
     * degrees ATDC. 
     */

    for (cycle = 1; cycle >= 0; cycle--)
    {
        index = find_closest_tooth(engine_cycle_angle, cycle);

        if (index >= 0)
        {
            break;
        }
    }

    if (cycle >= 0 && index >= 0)
    {
        event_st * const event = event_entry_get();

        if (event != NULL)
        {
            event->user_callback = callback;
            event->user_arg = user_arg;

            event_entry_list_insert(&context->event_callbacks[cycle][index], event);
        }
        /* Else we have a problem. */
    }
}

void trigger_36_1_handle_crank_pulse(trigger_wheel_36_1_context_st * const context,
                               uint32_t const timestamp)
{
    context->crank_trigger_state_handler(context, timestamp);
}

void trigger_36_1_handle_cam_pulse(trigger_wheel_36_1_context_st * const context,
                                     uint32_t const timestamp)
{
    context->cam_trigger_state_handler(context, timestamp);
}

float trigger_36_1_rpm_get(trigger_wheel_36_1_context_st * const context)
{
    return rpm_calculator_smoothed_rpm_get(context->rpm_calculator);
}

float trigger_36_1_crank_angle_get(trigger_wheel_36_1_context_st * const context)
{
    return context->angle_get_handler(context, false);
}

float trigger_36_1_engine_cycle_angle_get(trigger_wheel_36_1_context_st * const context)
{
    return context->angle_get_handler(context, true);
}


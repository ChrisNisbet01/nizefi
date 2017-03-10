#include "trigger_wheel_36_1.h"
#include "queue.h"

#include <stddef.h>

#define NUM_TEETH 35
#define NUM_MISSING_TEETH 1
#define TOTAL_TEETH (NUM_TEETH + NUM_MISSING_TEETH)

typedef enum trigger_wheel_state_t
{
    trigger_wheel_state_not_synched,
    trigger_wheel_state_synched
} trigger_wheel_state_t;

typedef void (*trigger_36_1_state_handler)(trigger_wheel_36_1_context_st * const context, uint32_t const timestamp);

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

    float tooth_angle; /* Rotation angle this tooth represents. */
    uint32_t last_timestamp;
    uint32_t last_delta; /* Difference in timestamp between this tooth and the previous timestamp.
                            Valid only if pulse_counter > 1.
                          */
    event_st event;
} tooth_context_st;

struct trigger_wheel_36_1_context_st
{
    trigger_wheel_state_t state;
    trigger_36_1_state_handler state_hander;

    uint32_t pulse_counter;
    uint32_t revolution_counter;

    size_t current_tooth; /* Valid only when synched. */
    size_t previous_tooth; /* (current_tooth - 1) % num_teeth. */
    tooth_context_st tooth_contexts[NUM_TEETH];

    tooth_context_st * tooth_1; /* When synched, this point to the entry for tooth #1, 
                                    which is the tooth after the missing tooth. 
                                 */
    tooth_context_st * tooth_next;

    CIRCLEQ_HEAD(,tooth_context_st)teeth_queue;
};

static void trigger_wheel_state_not_synched_handler(trigger_wheel_36_1_context_st * const context, uint32_t const timestamp);
static void trigger_wheel_state_synched_handler(trigger_wheel_36_1_context_st * const context, uint32_t const timestamp);

static trigger_wheel_36_1_context_st trigger_wheel_context;
static float const degrees_per_tooth = 360.0 / TOTAL_TEETH;

static void trigger_wheel_state_not_synched_handler(trigger_wheel_36_1_context_st * const context, uint32_t const timestamp)
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
            context->state = trigger_wheel_state_synched;
            context->state_hander = trigger_wheel_state_synched_handler;
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
        }
    }

    context->tooth_next = CIRCLEQ_NEXT(current_tooth, entry);
}

static void trigger_wheel_state_synched_handler(trigger_wheel_36_1_context_st * const context, uint32_t const timestamp)
{
    tooth_context_st * current_tooth = context->tooth_next; 
    tooth_context_st * previous_tooth = CIRCLEQ_PREV(current_tooth, entry);

    context->pulse_counter++;
    current_tooth->last_timestamp = timestamp;
    current_tooth->last_delta = timestamp - previous_tooth->last_timestamp;

    if (current_tooth == context->tooth_1)
    {
        /* Just found tooth #1. */
        debug_injector_pulse();
        context->revolution_counter++;
    }

    context->tooth_next = CIRCLEQ_NEXT(current_tooth, entry);
}

trigger_wheel_36_1_context_st * trigger_36_1_init(void)
{
    trigger_wheel_36_1_context_st * context = &trigger_wheel_context;
    size_t index;
    float tooth_angle;

    context->state = trigger_wheel_state_not_synched;
    context->state_hander = trigger_wheel_state_not_synched_handler;
    context->pulse_counter = 0;
    context->revolution_counter = 0;

    CIRCLEQ_INIT(&context->teeth_queue);
    for (index = 0; index < NUM_TEETH; index++)
    {
        tooth_context_st * const tooth_context = &context->tooth_contexts[index];
        tooth_context->tooth_angle = degrees_per_tooth * index;

        CIRCLEQ_INSERT_TAIL(&context->teeth_queue, tooth_context, entry);
    }
    context->tooth_next = CIRCLEQ_FIRST(&context->teeth_queue);
    context->tooth_1 = NULL;

    return context;
}

void trigger_36_1_handle_pulse(trigger_wheel_36_1_context_st * const context, uint32_t const timestamp)
{
    context->state_hander(context, timestamp);
}

# 09 - Work Queue Solution (Button Debouncing)

This example demonstrates using Zephyr's work queue mechanism to handle button debouncing by offloading interrupt work to a thread context. It shows proper ISR-to-thread communication using delayed work items.

## Overview

This application showcases:
- GPIO interrupt handling (button press detection)
- Offloading ISR work to thread context using work queues
- Delayed work items for button debouncing
- `k_work_delayable` API for scheduled work
- Proper ISR design (minimal ISR, heavy work in thread)

## The Problem: Button Bouncing

When a mechanical button is pressed, the contacts don't make a clean connection immediately. They "bounce" for several milliseconds:

```
Button Signal:
Press ↓
     _____     _   _   ___________________
    |     |   | | | | |
____|     |___| |_| |_|                    (Bouncing)
         ← Bouncing period ~50ms →
```

**Without debouncing:** Multiple interrupts fire, causing multiple false detections.

**With debouncing:** Wait 50ms after first edge, then read the stable value.

## How It Works

### Architecture

```
User presses button
       ↓
GPIO Interrupt (ISR)
       ↓
Schedule delayed work (50ms)
       ↓
Return from ISR (fast!)
       ↓
[50ms delay - bouncing settles]
       ↓
Work handler executes in thread context
       ↓
Read stable button state
       ↓
Process button press
```

### Key Components

**1. Delayable Work Item**
```c
static struct k_work_delayable button_work;
```
A work item that can be scheduled to run after a delay.

**2. Work Initialization**
```c
k_work_init_delayable(&button_work, button_work_handler);
```
Connects the work item to its handler function.

**3. ISR (Interrupt Service Routine)**
```c
void button_isr(const struct device *dev, 
                struct gpio_callback *cb, 
                uint32_t pins)
{
    // Schedule work to run after debounce delay
    k_work_reschedule(&button_work, K_MSEC(DEBOUNCE_DELAY_MS));
}
```
- **Fast ISR**: Just schedules work and returns
- **Minimal processing**: No delays, no heavy operations
- **ISR-safe**: `k_work_reschedule()` is safe to call from ISR

**4. Work Handler (Thread Context)**
```c
void button_work_handler(struct k_work *work)
{
    // Read button state after debounce delay
    int state = gpio_pin_get_dt(&btn);
    
    if (state > 0) {
        printk("Button pressed (debounced)!\r\n");
        // Do heavy processing here - we're in thread context!
    }
}
```
- Runs in system work queue thread
- Can call blocking functions
- Can do heavy processing
- Safe to use kernel APIs

## Hardware Requirements

- ESP32-S3 DevKit-C (or compatible board)
- Button connected to GPIO5 (D5)
- Button wired: GPIO5 → Button → GND (active low with pull-up)

## Devicetree Configuration

```dts
/ {
    aliases {
        my-button = &button_1;
    };

    buttons {
        compatible = "gpio-keys";
        
        button_1: d5 {
            gpios = <&gpio0 5 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
        };
    };
};
```

- **GPIO_ACTIVE_LOW**: Button is active when pulled to ground
- **GPIO_PULL_UP**: Internal pull-up resistor enabled

## Code Flow

### Initialization Phase

1. Initialize the delayable work item
2. Check if GPIO is ready
3. Configure GPIO pin as input
4. Configure interrupt (edge to active = falling edge for active-low)
5. Register callback function
6. Enter idle loop

### Runtime Phase

1. **User presses button**
2. **Hardware triggers interrupt** (falling edge detected)
3. **ISR executes** (`button_isr`)
   - Schedules work with 50ms delay
   - Returns immediately (ISR completes in microseconds)
4. **Button bounces** (for ~10-50ms)
5. **Timer expires** (50ms elapsed)
6. **Work handler executes** (`button_work_handler`)
   - Reads stable button state
   - Processes button press
   - Can do heavy work safely
7. **Repeat** on next button press

## Why Use Work Queues?

### ISR Constraints

❌ **Can't do in ISR:**
- Long delays (`k_msleep`, `k_sleep`)
- Blocking operations
- Heavy processing
- Mutex locking (only spinlocks)
- Memory allocation
- Printing large amounts

✅ **Can do in ISR:**
- Quick flag setting
- Schedule work
- Semaphore give
- Light processing (<10µs)

### Work Queue Benefits

✅ **Can do in work handler:**
- Blocking operations (mutexes, semaphores)
- Long processing
- Memory allocation
- File I/O
- Network operations
- Any kernel API

## Work Queue Types

### 1. System Work Queue (Used Here)

```c
k_work_schedule(&button_work, K_MSEC(50));
```

- **Default work queue** managed by kernel
- Priority: 10 (configurable with `CONFIG_SYSTEM_WORKQUEUE_PRIORITY`)
- Stack size: Configurable with `CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE`
- Shared by all system work items
- **Pros**: Simple, no setup needed
- **Cons**: Shared resource, can be delayed by other work

### 2. Custom Work Queue

```c
// Define custom work queue
K_THREAD_STACK_DEFINE(my_wq_stack, 1024);
struct k_work_q my_work_q;

// Initialize
k_work_queue_start(&my_work_q, 
                   my_wq_stack,
                   K_THREAD_STACK_SIZEOF(my_wq_stack),
                   5,      // Priority
                   NULL);  // Config

// Submit to custom queue
k_work_queue_schedule(&my_work_q, &button_work, K_MSEC(50));
```

- **Dedicated thread** for your work items
- Custom priority and stack size
- Isolated from system work
- **Pros**: Guaranteed timing, custom priority
- **Cons**: More memory, needs initialization

## Work Item Types

### 1. Simple Work (k_work)

**For immediate execution:**

```c
// Define work
void my_work_handler(struct k_work *work) {
    printk("Work executed!\n");
}

K_WORK_DEFINE(my_work, my_work_handler);

// Submit from ISR or thread
k_work_submit(&my_work);  // Executes ASAP
```

**Use when**: No delay needed, execute immediately

### 2. Delayable Work (k_work_delayable)

**For delayed/scheduled execution (Used in this example):**

```c
// Define delayable work
void delayed_handler(struct k_work *work) {
    printk("Delayed work executed!\n");
}

K_WORK_DELAYABLE_DEFINE(my_delayed_work, delayed_handler);

// Schedule with delay
k_work_schedule(&my_delayed_work, K_MSEC(500));

// Reschedule (cancels previous, schedules new)
k_work_reschedule(&my_delayed_work, K_MSEC(100));

// Cancel if pending
k_work_cancel_delayable(&my_delayed_work);
```

**Use when**: Need delay, debouncing, periodic tasks

### 3. Pollable Work (k_work_poll)

**For waiting on multiple events:**

```c
// Advanced: wait on semaphores, FIFOs, etc.
struct k_work_poll my_poll_work;
struct k_poll_event events[2];

k_work_poll_init(&my_poll_work, poll_handler);
k_work_poll_submit(&my_poll_work, events, 2, K_FOREVER);
```

**Use when**: Complex multi-source event handling

## API Reference

### Initialization

```c
// Initialize simple work
K_WORK_DEFINE(work_name, handler_func);
// Or at runtime:
k_work_init(&work_item, handler_func);

// Initialize delayable work
K_WORK_DELAYABLE_DEFINE(work_name, handler_func);
// Or at runtime:
k_work_init_delayable(&work_item, handler_func);
```

### Submission

```c
// Submit simple work (immediate)
k_work_submit(&work_item);                    // To system queue
k_work_submit_to_queue(&my_queue, &work);    // To custom queue

// Submit delayable work (scheduled)
k_work_schedule(&delayable_work, delay);      // To system queue
k_work_reschedule(&delayable_work, delay);    // Cancel + reschedule
k_work_schedule_for_queue(&my_queue, &work, delay);  // To custom queue
```

### Cancellation

```c
// Cancel simple work
int ret = k_work_cancel(&work_item);
// Returns: 0 if cancelled, -EINPROGRESS if running

// Cancel delayable work
int ret = k_work_cancel_delayable(&delayable_work);
// Returns: 0 if cancelled, -EINPROGRESS if running/pending

// Cancel and wait for completion
k_work_cancel_sync(&work_item, &sync_obj);
```

### Status Query

```c
// Check if work is pending
bool pending = k_work_is_pending(&work_item);

// Check if delayable work is pending
bool pending = k_work_delayable_is_pending(&delayable_work);

// Get remaining delay
k_ticks_t remaining = k_work_delayable_remaining_get(&delayable_work);
```

## Debouncing Strategies

### Strategy 1: Delay After First Edge (This Example)

```c
void button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_work_reschedule(&button_work, K_MSEC(50));
}
```

**Pros**: Simple, works for most cases  
**Cons**: Multiple presses during delay period merge into one

### Strategy 2: Ignore Edges During Debounce

```c
static bool debouncing = false;

void button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (debouncing) return;  // Ignore additional edges
    
    debouncing = true;
    k_work_schedule(&button_work, K_MSEC(50));
}

void button_work_handler(struct k_work *work)
{
    int state = gpio_pin_get_dt(&btn);
    if (state > 0) {
        printk("Button pressed!\n");
    }
    debouncing = false;  // Re-enable detection
}
```

**Pros**: Prevents duplicate detection  
**Cons**: More complex, misses rapid presses

### Strategy 3: State Machine

```c
static int last_state = 0;
static uint32_t last_change_time = 0;

void button_work_handler(struct k_work *work)
{
    int current_state = gpio_pin_get_dt(&btn);
    uint32_t now = k_uptime_get_32();
    
    // Only process if state changed and stable for 50ms
    if (current_state != last_state && 
        (now - last_change_time) >= 50) {
        
        if (current_state) {
            printk("Button pressed!\n");
        } else {
            printk("Button released!\n");
        }
        
        last_state = current_state;
        last_change_time = now;
    }
}
```

**Pros**: Detects press and release, very robust  
**Cons**: Most complex

## Building and Running

```bash
# Navigate to the project directory
cd apps/09_solution_workqueue

# Build for ESP32-S3
west build -b esp32s3_devkitc/esp32s3/procpu

# Flash to device
west flash

# Monitor output
west espressif monitor
```

## Expected Output

```
*** Booting Zephyr OS build v3.x.x ***

(Press button)
Doing some work...now with debounce!

(Press button rapidly multiple times)
Doing some work...now with debounce!

(Release and press again)
Doing some work...now with debounce!
```

Each button press, no matter how bouncy, produces only one message.

## Learning Objectives

After completing this example, you should understand:
1. Why ISRs must be short and fast
2. How to offload work from ISR to thread context
3. Using work queues for deferred processing
4. Implementing button debouncing with delayed work
5. Difference between simple and delayable work items
6. When to use system vs. custom work queues
7. Proper ISR design patterns in embedded systems

## Common Pitfalls

### ❌ Doing Heavy Work in ISR

```c
void button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    // DON'T DO THIS!
    k_msleep(50);  // Blocks interrupts!
    int state = gpio_pin_get_dt(&btn);
    process_button(state);  // Too slow!
}
```

### ❌ Forgetting to Initialize Work

```c
static struct k_work_delayable button_work;

int main(void) {
    // FORGOT: k_work_init_delayable(&button_work, handler);
    k_work_schedule(&button_work, K_MSEC(50));  // Will crash!
}
```

### ✅ Correct Pattern

```c
static struct k_work_delayable button_work;

int main(void) {
    k_work_init_delayable(&button_work, button_work_handler);  // Initialize!
    // ... rest of setup ...
}

void button_isr(...) {
    k_work_reschedule(&button_work, K_MSEC(50));  // Just schedule
}

void button_work_handler(struct k_work *work) {
    // Do all heavy work here
}
```

## Advanced Patterns

### Pattern 1: Passing Data to Work Handler

```c
struct button_work_data {
    struct k_work_delayable work;
    int button_id;
    uint32_t press_time;
};

void button_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct button_work_data *data = 
        CONTAINER_OF(dwork, struct button_work_data, work);
    
    printk("Button %d pressed at %u ms\n", 
           data->button_id, data->press_time);
}

// In ISR
void button_isr(...) {
    work_data.button_id = 1;
    work_data.press_time = k_uptime_get_32();
    k_work_reschedule(&work_data.work, K_MSEC(50));
}
```

### Pattern 2: Periodic Work

```c
void periodic_work_handler(struct k_work *work) {
    // Do periodic task
    read_sensors();
    
    // Reschedule for next period
    k_work_reschedule(&periodic_work, K_SECONDS(1));
}

// Start periodic work
k_work_init_delayable(&periodic_work, periodic_work_handler);
k_work_schedule(&periodic_work, K_SECONDS(1));
```

### Pattern 3: Work Queue Chain

```c
K_WORK_DEFINE(stage1_work, stage1_handler);
K_WORK_DEFINE(stage2_work, stage2_handler);

void stage1_handler(struct k_work *work) {
    printk("Stage 1\n");
    k_work_submit(&stage2_work);  // Trigger next stage
}

void stage2_handler(struct k_work *work) {
    printk("Stage 2\n");
}
```

## Exercises

1. **Variable Debounce**: Make debounce delay configurable via Kconfig
2. **Long Press**: Detect long button press (>1 second)
3. **Double Click**: Detect double-click within 300ms
4. **Multiple Buttons**: Handle 3 buttons with separate work items
5. **Custom Queue**: Create a high-priority custom work queue
6. **Statistics**: Count total button presses and print every 10 seconds

## Debugging Tips

- Enable work queue debugging: `CONFIG_INIT_STACKS=y`
- Monitor system work queue: Check stack usage
- Add timestamps to handler to measure latency
- Use `printk` with ISR-safe format in ISR
- Check `k_work_is_pending()` if work seems to disappear

## Performance Metrics

- **ISR latency**: ~1-5 µs (just scheduling work)
- **Debounce delay**: 50 ms (configurable)
- **Work handler latency**: ~1-10 ms (depends on system load)
- **Memory per work item**: ~24-32 bytes

## Next Steps

- See `09_demo_gpio_interrupt` for basic GPIO interrupts
- See `09_demo_timer` for timer-based work scheduling
- Learn about poll API for complex event handling
- Explore `k_timer` for high-precision periodic tasks

## References

- [Zephyr Work Queue Documentation](https://docs.zephyrproject.org/latest/kernel/services/threads/workqueue.html)
- [Zephyr GPIO Documentation](https://docs.zephyrproject.org/latest/hardware/peripherals/gpio.html)
- [ISR Best Practices](https://docs.zephyrproject.org/latest/kernel/services/interrupts.html)

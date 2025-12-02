# 08 - Multithreading Demo

This example demonstrates basic multithreading in Zephyr RTOS by running two concurrent tasks: one thread blinks an LED while the main thread continuously prints messages.

## Overview

This application showcases:
- Creating and starting a thread using `k_thread_create()`
- Using thread stacks with `K_THREAD_STACK_DEFINE()`
- Running concurrent tasks independently
- LED GPIO control in a separate thread

## Hardware Requirements

- ESP32-S3 DevKit-C (or compatible board)
- LED connected to the GPIO defined as `my_led` alias in devicetree

## How It Works

### Thread Architecture

The application has two execution contexts:

1. **Main Thread** (implicit)
   - Priority: 0 (cooperative)
   - Prints "Hello" every 700ms
   - Runs in the main context

2. **Blink Thread** (explicit)
   - Priority: 7 (preemptive)
   - Toggles LED every 500ms
   - Runs independently in its own stack

### Thread Creation

```c
k_tid_t blink_tid = k_thread_create(
    &blink_thread,              // Thread control block
    blink_stack,                // Stack memory
    K_THREAD_STACK_SIZEOF(blink_stack),  // Stack size
    blink_thread_start,         // Entry function
    NULL, NULL, NULL,           // Arguments (unused)
    7,                          // Priority (lower number = higher priority)
    0,                          // Options
    K_NO_WAIT                   // Start immediately
);
```

### Thread Creation Parameters Explained

#### 1. Thread Control Block (`&blink_thread`)
- Kernel structure that tracks thread state
- Must be allocated before calling `k_thread_create()`
- Contains scheduling info, priority, stack pointer, etc.

#### 2. Stack Memory (`blink_stack`)
- Pre-allocated memory for thread's stack
- Each thread needs its own stack
- Must be defined with `K_THREAD_STACK_DEFINE()`

#### 3. Stack Size (`K_THREAD_STACK_SIZEOF(blink_stack)`)
- Size of the stack in bytes
- Includes guard areas and alignment
- Too small = stack overflow, too large = wasted RAM

#### 4. Entry Function (`blink_thread_start`)
- The function that runs when thread starts
- Signature: `void func(void *p1, void *p2, void *p3)`
- Should typically never return (infinite loop)

#### 5. Arguments (`NULL, NULL, NULL`)
- Three void pointers passed to entry function
- Can pass data, structures, or handles to thread
- Example: passing sensor device handle
```c
// Pass sensor handle to thread
k_thread_create(&sensor_thread, stack, size,
                sensor_entry,
                (void *)sensor_device,  // arg1: sensor handle
                (void *)&config,        // arg2: config struct
                NULL,                   // arg3: unused
                7, 0, K_NO_WAIT);
```

#### 6. Priority (Integer: -16 to 15 for preemptive, 0 to -15 for cooperative)

**Priority Ranges:**

| Range | Type | Behavior | Use Case |
|-------|------|----------|----------|
| **-16 to -1** | Meta-IRQ | Highest priority, cooperative | Critical tasks |
| **0 to 15** | Preemptive | Can be interrupted by higher priority | Normal tasks |
| **-1 to -15** | Cooperative | Only yields voluntarily | Background tasks |

**Priority Rules:**
- **Lower number = Higher priority** (0 > 1 > 2 ... > 15)
- Negative priorities are cooperative (only yield with `k_yield()` or blocking calls)
- Priority 0-15: Preemptive threads (scheduler can interrupt them)
- Most user threads: 7-10 (medium priority)

**Examples:**
```c
// High priority preemptive thread
k_thread_create(..., 0, 0, K_NO_WAIT);  // Highest preemptive

// Normal priority preemptive thread
k_thread_create(..., 7, 0, K_NO_WAIT);  // Typical application thread

// Low priority preemptive thread
k_thread_create(..., 15, 0, K_NO_WAIT); // Lowest preemptive

// Cooperative thread
k_thread_create(..., -5, 0, K_NO_WAIT); // Cooperative, won't be preempted
```

#### 7. Options (Bitwise OR of flags)

**Common Options:**

| Option | Value | Description |
|--------|-------|-------------|
| `0` | Default | No special options |
| `K_ESSENTIAL` | 0x01 | System panics if thread terminates |
| `K_FP_REGS` | 0x02 | Thread uses floating-point registers |
| `K_USER` | 0x04 | Thread runs in user mode (memory protection) |
| `K_INHERIT_PERMS` | 0x08 | Inherit permissions from parent |

**Examples:**
```c
// No options (most common)
k_thread_create(..., 7, 0, K_NO_WAIT);

// Essential thread (critical system thread)
k_thread_create(..., 7, K_ESSENTIAL, K_NO_WAIT);

// Thread using floating-point math
k_thread_create(..., 7, K_FP_REGS, K_NO_WAIT);

// User mode thread (with memory protection)
k_thread_create(..., 7, K_USER, K_NO_WAIT);

// Multiple options (bitwise OR)
k_thread_create(..., 7, K_ESSENTIAL | K_FP_REGS, K_NO_WAIT);
```

**Option Details:**

- **`K_ESSENTIAL`**: If thread exits/aborts, kernel panics (use for critical threads)
- **`K_FP_REGS`**: Saves/restores FPU registers on context switch (adds overhead)
- **`K_USER`**: Requires `CONFIG_USERSPACE=y`, thread has restricted privileges
- **`K_INHERIT_PERMS`**: Inherits memory domain from creating thread

#### 8. Delay/Start Timing

**When the thread starts executing:**

| Value | Behavior | Use Case |
|-------|----------|----------|
| `K_NO_WAIT` | Start immediately | Most common, thread ready to run |
| `K_FOREVER` | Never auto-start | Manual start with `k_thread_start()` |
| `K_MSEC(ms)` | Delay in milliseconds | Delayed start |
| `K_SECONDS(s)` | Delay in seconds | Delayed start |
| `K_TICKS(t)` | Delay in system ticks | Precise timing |

**Examples:**
```c
// Start immediately (thread becomes ready)
k_thread_create(..., K_NO_WAIT);

// Start after 1 second
k_thread_create(..., K_SECONDS(1));

// Start after 500ms
k_thread_create(..., K_MSEC(500));

// Create suspended, start manually later
k_tid_t tid = k_thread_create(..., K_FOREVER);
// ... do other initialization ...
k_thread_start(tid);  // Start it now

// Start after 100 ticks
k_thread_create(..., K_TICKS(100));
```

**Important Notes:**
- `K_NO_WAIT` doesn't mean thread runs immediately—it's added to ready queue
- Higher priority threads run first
- If current thread has higher priority, new thread waits
- Delay starts counting from `k_thread_create()` call

### Complete Examples

**Example 1: High-Priority Sensor Thread**
```c
k_tid_t sensor_tid = k_thread_create(
    &sensor_thread,                    // Control block
    sensor_stack,                      // Stack
    K_THREAD_STACK_SIZEOF(sensor_stack),
    sensor_entry,                      // Entry function
    (void *)&sensor_dev,               // Pass sensor device
    NULL,                              // No arg2
    NULL,                              // No arg3
    5,                                 // High priority (preemptive)
    K_FP_REGS,                        // Uses floating-point math
    K_NO_WAIT                          // Start immediately
);
```

**Example 2: Low-Priority Background Thread**
```c
k_tid_t bg_tid = k_thread_create(
    &background_thread,
    bg_stack,
    K_THREAD_STACK_SIZEOF(bg_stack),
    background_task,
    NULL, NULL, NULL,
    12,                                // Low priority
    0,                                 // No special options
    K_SECONDS(5)                       // Start after 5 seconds
);
```

**Example 3: Critical System Thread**
```c
k_tid_t watchdog_tid = k_thread_create(
    &watchdog_thread,
    watchdog_stack,
    K_THREAD_STACK_SIZEOF(watchdog_stack),
    watchdog_entry,
    NULL, NULL, NULL,
    0,                                 // Highest preemptive priority
    K_ESSENTIAL,                       // Panic if thread dies
    K_NO_WAIT
);
```

### Key Concepts

**Thread Stack**
```c
#define BLINK_THREAD_STACK_SIZE 256
K_THREAD_STACK_DEFINE(blink_stack, BLINK_THREAD_STACK_SIZE);
```
Each thread needs its own stack memory for local variables and function calls.

**Thread Data Structure**
```c
static struct k_thread blink_thread;
```
The kernel uses this to track thread state, priority, and scheduling information.

**Thread Priority**
- Lower numbers = higher priority
- Priority 7 is preemptive (can interrupt cooperative threads)
- Priority 0 (main) is cooperative (yields voluntarily)

## Code Flow

1. **Initialization**
   - Check GPIO is ready
   - Configure LED pin as output
   - Create and start blink thread

2. **Concurrent Execution**
   - Blink thread: Toggle LED → Sleep 500ms → Repeat
   - Main thread: Print "Hello" → Sleep 700ms → Repeat

3. **Independent Timing**
   - Both threads run at different rates
   - The scheduler switches between them automatically

## Building and Running

```bash
# Navigate to the project directory
cd apps/08_demo_multithreading

# Build for ESP32-S3
west build -b esp32s3_devkitc/esp32s3/procpu

# Flash to device
west flash

# Monitor output
west espressif monitor
```

## Expected Output

```
Starting blink thread
Hello
Hello
Hello
Hello
...
```

You should see:
- LED blinking at ~2 Hz (500ms on, 500ms off)
- "Hello" messages appearing at ~1.4 Hz (every 700ms)
- Both happening simultaneously

## Learning Objectives

After completing this example, you should understand:
1. How to create and start threads in Zephyr
2. How to allocate stack memory for threads
3. Thread priority and scheduling concepts
4. Running multiple independent tasks concurrently
5. Basic GPIO operations in a threaded context

## Exercises

1. **Change Timing**: Modify the sleep intervals to see how it affects execution
2. **Add More Threads**: Create a third thread that does something different
3. **Priority Experiment**: Change thread priorities and observe the behavior
4. **Stack Size**: Try reducing the stack size until the program fails
5. **Thread Arguments**: Pass parameters to the blink thread (e.g., blink rate)

## Next Steps

- See `08_demo_mutex` to learn about thread synchronization
- See `08_solution_sensor_queue` to learn about inter-thread communication

## References

- [Zephyr Threading Documentation](https://docs.zephyrproject.org/latest/kernel/services/threads/index.html)
- [Zephyr Scheduling Documentation](https://docs.zephyrproject.org/latest/kernel/services/scheduling/index.html)

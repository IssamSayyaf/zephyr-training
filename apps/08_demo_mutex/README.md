# 08 - Mutex Demo

This example demonstrates mutex (mutual exclusion) usage in Zephyr RTOS to protect shared data between threads. An input thread adjusts the blink rate, and a blink thread uses that rate—both safely accessing a shared variable using a mutex.

## Overview

This application showcases:
- Creating and using mutexes with `K_MUTEX_DEFINE()`
- Protecting shared variables with `k_mutex_lock()` and `k_mutex_unlock()`
- Inter-thread communication through shared memory
- Console input handling in a thread
- Race condition prevention

## Hardware Requirements

- ESP32-S3 DevKit-C (or compatible board)
- LED connected to the GPIO defined as `my_led` alias in devicetree
- Serial console connection (USB)

## The Problem: Race Conditions

Without synchronization, this scenario could occur:

```
Time    Blink Thread              Input Thread
----    ------------              ------------
t1      Read blink_sleep_ms (500)
t2                                Read blink_sleep_ms (500)
t3                                Add 100 → 600
t4                                Write blink_sleep_ms = 600
t5      Add 0 → 500
t6      Write blink_sleep_ms = 500  ← Lost update!
```

The mutex prevents this by ensuring only one thread accesses the variable at a time.

## How It Works

### Thread Architecture

The application has three execution contexts:

1. **Main Thread**
   - Priority: 0 (preemptive, highest)
   - Initializes hardware and threads
   - Idle loop

2. **Input Thread**
   - Priority: 7 (preemptive, higher)
   - Options: 0 (no special options)
   - Start: `K_NO_WAIT` (immediately)
   - Waits for console input (blocking)
   - Interprets '+' or '-' commands
   - Updates `blink_sleep_ms` with mutex protection

3. **Blink Thread**
   - Priority: 8 (preemptive, lower priority than input)
   - Options: 0 (no special options)
   - Start: `K_NO_WAIT` (immediately)
   - Reads `blink_sleep_ms` with mutex protection
   - Toggles LED at the current rate

**Priority Hierarchy:** Input (7) > Blink (8) means input thread can preempt blink thread

### Mutex Usage Pattern

**Defining a Mutex**
```c
K_MUTEX_DEFINE(my_mutex);
```

**Protecting Shared Data**
```c
// Lock the mutex (wait forever if needed)
k_mutex_lock(&my_mutex, K_FOREVER);

// Critical section: safely access shared variable
blink_sleep_ms += 100;

// Unlock the mutex
k_mutex_unlock(&my_mutex);
```

### Shared Variable

```c
static int32_t blink_sleep_ms = 500;  // Shared between threads
```

Both threads access this variable, but **always** within mutex protection to prevent race conditions.

## Code Flow

### Input Thread Flow

1. Wait for console input (blocking call)
2. Check if first character is '+' or '-'
3. **Lock mutex**
4. Update `blink_sleep_ms` (increment/decrement by 100ms)
5. Clamp value between 0 and 2000ms
6. **Unlock mutex**
7. Print new value
8. Repeat

### Blink Thread Flow

1. **Lock mutex**
2. Copy `blink_sleep_ms` to local variable
3. **Unlock mutex**
4. Toggle LED
5. Sleep for the copied duration
6. Repeat

### Why Copy the Value?

```c
k_mutex_lock(&my_mutex, K_FOREVER);
sleep_ms = blink_sleep_ms;  // Copy to local variable
k_mutex_unlock(&my_mutex);

k_msleep(sleep_ms);  // Sleep outside critical section
```

This minimizes the time the mutex is held. Sleeping with the mutex locked would block other threads unnecessarily.

## User Interaction

### Commands

- Type `+` and press Enter: Increase blink rate by 100ms (slower)
- Type `-` and press Enter: Decrease blink rate by 100ms (faster)
- Any other input: Ignored

### Bounds

- Minimum: 0ms (LED stays on/off rapidly)
- Maximum: 2000ms (2 seconds between toggles)

## Building and Running

```bash
# Navigate to the project directory
cd apps/08_demo_mutex

# Build for ESP32-S3
west build -b esp32s3_devkitc/esp32s3/procpu

# Flash to device
west flash

# Monitor output (interactive console)
west espressif monitor
```

## Expected Output

```
Starting input thread
Starting blink thread
+
Updating blink sleep to: 600
+
Updating blink sleep to: 700
-
Updating blink sleep to: 600
-
Updating blink sleep to: 500
```

You should observe:
- LED blink rate changes based on your input
- Thread-safe updates to the shared variable
- No race conditions or data corruption

## Key Concepts

### Mutex (Mutual Exclusion)

A mutex is a locking mechanism that ensures only one thread can access a shared resource at a time.

**When to use mutexes:**
- Protecting shared variables accessed by multiple threads
- Short critical sections (microseconds to milliseconds)
- When you need ownership semantics

**When NOT to use mutexes:**
- Inside ISRs (use spinlocks instead)
- For long operations (consider semaphores or other mechanisms)
- When threads don't actually share data

### Critical Section

The code between `k_mutex_lock()` and `k_mutex_unlock()` is called a critical section. Keep it as short as possible to avoid blocking other threads.

### Blocking Behavior

```c
k_mutex_lock(&my_mutex, K_FOREVER);  // Wait indefinitely
k_mutex_lock(&my_mutex, K_NO_WAIT);  // Return immediately if locked
k_mutex_lock(&my_mutex, K_MSEC(100)); // Wait up to 100ms
```

## Semaphores: An Alternative to Mutexes

### What is a Semaphore?

A semaphore is a signaling mechanism that controls access to resources using a counter. Unlike mutexes, semaphores don't have ownership—any thread can signal (give) a semaphore.

### Types of Semaphores

#### 1. Binary Semaphore (0 or 1)

Acts like a signal or flag. Similar to mutex but without ownership.

**Definition:**
```c
K_SEM_DEFINE(my_binary_sem, 0, 1);  // Initial: 0, Max: 1
                                     // Starts unavailable
```

**Usage:**
```c
// Thread 1: Wait for signal
k_sem_take(&my_binary_sem, K_FOREVER);  // Blocks until signaled
printk("Received signal!\n");

// Thread 2: Send signal
k_sem_give(&my_binary_sem);  // Wakes up Thread 1
```

**Common Use Cases:**
- Thread synchronization (one thread signals another)
- ISR to thread communication
- Event notification
- Resource availability signaling

**Example: ISR to Thread Communication**
```c
K_SEM_DEFINE(data_ready_sem, 0, 1);

// ISR signals data is ready
void my_isr(void) {
    // ... read data from hardware ...
    k_sem_give(&data_ready_sem);  // Signal thread
}

// Thread waits for data
void processing_thread(void) {
    while (1) {
        k_sem_take(&data_ready_sem, K_FOREVER);  // Wait for ISR
        process_data();
    }
}
```

#### 2. Counting Semaphore (0 to N)

Tracks multiple identical resources or counts events.

**Definition:**
```c
K_SEM_DEFINE(resource_sem, 3, 3);  // Initial: 3, Max: 3
                                    // 3 resources available
```

**Usage:**
```c
// Acquire a resource
if (k_sem_take(&resource_sem, K_MSEC(100)) == 0) {
    // Got a resource, use it
    use_resource();
    
    // Release the resource
    k_sem_give(&resource_sem);
} else {
    printk("No resources available\n");
}
```

**Common Use Cases:**
- Managing pool of identical resources (buffers, connections)
- Limiting concurrent access (max N threads)
- Counting events (accumulating signals)
- Producer-consumer with resource limiting

**Example: Buffer Pool Management**
```c
#define NUM_BUFFERS 5
K_SEM_DEFINE(buffer_sem, NUM_BUFFERS, NUM_BUFFERS);

char buffers[NUM_BUFFERS][256];

// Acquire a buffer
char* get_buffer(void) {
    if (k_sem_take(&buffer_sem, K_MSEC(100)) == 0) {
        // Find and return an available buffer
        return find_free_buffer();
    }
    return NULL;  // No buffers available
}

// Release a buffer
void put_buffer(char *buf) {
    k_sem_give(&buffer_sem);  // Increment available count
}
```

**Example: Rate Limiting**
```c
// Allow max 3 concurrent downloads
K_SEM_DEFINE(download_sem, 3, 3);

void download_thread(void) {
    k_sem_take(&download_sem, K_FOREVER);  // Wait for slot
    
    // Download file (only 3 can run simultaneously)
    download_file();
    
    k_sem_give(&download_sem);  // Release slot
}
```

### Mutex vs. Semaphore: Key Differences

| Feature | Mutex | Binary Semaphore | Counting Semaphore |
|---------|-------|------------------|-------------------|
| **Purpose** | Mutual exclusion | Signaling/Synchronization | Resource counting |
| **Initial Value** | Unlocked (1) | Usually 0 or 1 | 0 to N |
| **Ownership** | ✅ Yes (thread that locks must unlock) | ❌ No (any thread can give/take) | ❌ No |
| **Priority Inheritance** | ✅ Yes (prevents priority inversion) | ❌ No | ❌ No |
| **Use in ISR** | ❌ No (lock only) | ✅ Yes (give only) | ✅ Yes (give only) |
| **Recursive Locking** | ✅ Yes (same thread can lock multiple times) | ❌ No | ❌ No |
| **Use Case** | Protect shared data | Event notification | Resource pool management |
| **Max Count** | 1 | 1 | N (configurable) |

### When to Use What?

#### Use Mutex When:
✅ Protecting shared memory/variables  
✅ Need ownership semantics (who locked it?)  
✅ Need priority inheritance  
✅ Thread-to-thread synchronization  
✅ Short critical sections  

```c
// Protecting shared counter
K_MUTEX_DEFINE(counter_mutex);
int shared_counter = 0;

k_mutex_lock(&counter_mutex, K_FOREVER);
shared_counter++;  // Protected access
k_mutex_unlock(&counter_mutex);
```

#### Use Binary Semaphore When:
✅ ISR to thread signaling  
✅ Event notification between threads  
✅ Synchronizing thread start/completion  
✅ No ownership needed  
✅ One-way signaling  

```c
// ISR signals thread
K_SEM_DEFINE(event_sem, 0, 1);

// In ISR
k_sem_give(&event_sem);  // Signal event

// In thread
k_sem_take(&event_sem, K_FOREVER);  // Wait for event
```

#### Use Counting Semaphore When:
✅ Managing pool of resources  
✅ Limiting concurrent access  
✅ Counting events/signals  
✅ Producer-consumer scenarios  
✅ Rate limiting  

```c
// Limit to 5 concurrent operations
K_SEM_DEFINE(worker_sem, 5, 5);

k_sem_take(&worker_sem, K_FOREVER);  // Get slot
do_work();  // Max 5 threads here
k_sem_give(&worker_sem);  // Release slot
```

### Semaphore API Reference

#### Definition
```c
// Static definition
K_SEM_DEFINE(my_sem, initial_count, limit);

// Runtime initialization
struct k_sem my_sem;
k_sem_init(&my_sem, initial_count, limit);
```

#### Operations
```c
// Take (decrement, P operation, wait)
int ret = k_sem_take(&my_sem, timeout);
// Returns 0 on success, -EBUSY if timeout with K_NO_WAIT, -EAGAIN if timeout

// Give (increment, V operation, signal)
k_sem_give(&my_sem);  // Never blocks, can be called from ISR

// Count query
unsigned int count = k_sem_count_get(&my_sem);

// Reset to initial value
k_sem_reset(&my_sem);
```

#### Timeout Options
```c
k_sem_take(&my_sem, K_NO_WAIT);     // Don't wait, return immediately
k_sem_take(&my_sem, K_FOREVER);     // Wait indefinitely
k_sem_take(&my_sem, K_MSEC(100));   // Wait up to 100ms
k_sem_take(&my_sem, K_SECONDS(5));  // Wait up to 5 seconds
```

### Complete Example: Binary Semaphore

```c
#include <zephyr/kernel.h>

K_SEM_DEFINE(button_pressed, 0, 1);  // Binary semaphore

// Button ISR
void button_isr(void) {
    k_sem_give(&button_pressed);  // Signal button press
}

// Processing thread
void button_handler_thread(void) {
    while (1) {
        // Wait for button press
        k_sem_take(&button_pressed, K_FOREVER);
        
        printk("Button pressed! Processing...\n");
        // Handle button press
    }
}
```

### Complete Example: Counting Semaphore

```c
#include <zephyr/kernel.h>

#define MAX_CONNECTIONS 3
K_SEM_DEFINE(connection_pool, MAX_CONNECTIONS, MAX_CONNECTIONS);

void client_handler(void) {
    // Try to get a connection slot
    if (k_sem_take(&connection_pool, K_MSEC(1000)) == 0) {
        printk("Connection acquired (available: %d)\n", 
               k_sem_count_get(&connection_pool));
        
        // Process client request
        handle_client();
        
        // Release connection slot
        k_sem_give(&connection_pool);
        printk("Connection released\n");
    } else {
        printk("Server busy, no connections available\n");
    }
}
```

### Advanced Pattern: Mutex + Semaphore

Sometimes you need both: mutex for data protection, semaphore for signaling.

```c
K_MUTEX_DEFINE(data_mutex);
K_SEM_DEFINE(data_ready, 0, 1);
int shared_data = 0;

// Producer thread
void producer(void) {
    while (1) {
        // Produce data
        int new_data = read_sensor();
        
        // Protect shared data with mutex
        k_mutex_lock(&data_mutex, K_FOREVER);
        shared_data = new_data;
        k_mutex_unlock(&data_mutex);
        
        // Signal data is ready with semaphore
        k_sem_give(&data_ready);
        
        k_msleep(1000);
    }
}

// Consumer thread
void consumer(void) {
    while (1) {
        // Wait for signal
        k_sem_take(&data_ready, K_FOREVER);
        
        // Read shared data with mutex protection
        k_mutex_lock(&data_mutex, K_FOREVER);
        int data = shared_data;
        k_mutex_unlock(&data_mutex);
        
        // Process data
        printk("Received data: %d\n", data);
    }
}
```

### Priority Inversion Problem

**Mutex**: Has priority inheritance to prevent priority inversion  
**Semaphore**: Does NOT have priority inheritance

**Example of Priority Inversion:**
```
Low priority thread (L) takes semaphore
High priority thread (H) tries to take same semaphore → blocks
Medium priority thread (M) preempts L
Result: H waits for M to finish (priority inversion!)
```

**Solution**: Use mutex when priority inheritance is needed.

### Performance Comparison

| Operation | Mutex | Binary Semaphore | Counting Semaphore |
|-----------|-------|------------------|-------------------|
| Lock/Take | ~50-100 cycles | ~40-80 cycles | ~40-80 cycles |
| Unlock/Give | ~30-60 cycles | ~30-60 cycles | ~30-60 cycles |
| Memory | ~16-24 bytes | ~12-20 bytes | ~12-20 bytes |
| Priority Inheritance | Yes (overhead) | No | No |

*Values are approximate and platform-dependent*

## Learning Objectives

After completing this example, you should understand:
1. What race conditions are and why they're problematic
2. How to use mutexes to protect shared data
3. Proper mutex lock/unlock patterns
4. Minimizing critical section duration
5. Console input handling in Zephyr
6. Thread-safe programming practices

## Common Pitfalls

### ❌ Forgetting to Unlock

```c
k_mutex_lock(&my_mutex, K_FOREVER);
if (error) {
    return;  // DEADLOCK! Mutex never unlocked
}
k_mutex_unlock(&my_mutex);
```

### ❌ Deadlock with Multiple Mutexes

```c
// Thread 1
k_mutex_lock(&mutex_a, K_FOREVER);
k_mutex_lock(&mutex_b, K_FOREVER);  // Can deadlock

// Thread 2
k_mutex_lock(&mutex_b, K_FOREVER);
k_mutex_lock(&mutex_a, K_FOREVER);  // Can deadlock
```

### ✅ Always unlock, even on errors

```c
k_mutex_lock(&my_mutex, K_FOREVER);
// ... critical section ...
k_mutex_unlock(&my_mutex);  // Always reached
```

## Exercises

1. **Remove Mutex**: Comment out mutex locks and see if you can observe race conditions
2. **Add Counter**: Create a shared counter incremented by multiple threads
3. **Priority Inversion**: Research and experiment with priority inversion scenarios
4. **Multiple Variables**: Protect multiple related variables with the same mutex
5. **Timeout Handling**: Use `K_MSEC()` timeout and handle lock failure

## Debugging Tips

- Enable `CONFIG_DEBUG_THREAD_INFO=y` to see thread states
- Use `CONFIG_THREAD_MONITOR=y` for thread monitoring
- Add printk statements before/after mutex operations to trace execution
- Check for deadlocks if the program hangs

## Next Steps

- See `08_demo_multithreading` for basic threading concepts
- See `08_solution_sensor_queue` for lock-free inter-thread communication
- Learn about semaphores for more complex synchronization patterns

## References

- [Zephyr Mutex Documentation](https://docs.zephyrproject.org/latest/kernel/services/synchronization/mutexes.html)
- [Zephyr Synchronization Documentation](https://docs.zephyrproject.org/latest/kernel/services/synchronization/index.html)
- [Console API Documentation](https://docs.zephyrproject.org/latest/services/console/index.html)

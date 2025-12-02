# 08 - Sensor Queue Solution

This example demonstrates inter-thread communication using message queues in Zephyr RTOS. A sensor thread reads temperature data and puts it in a queue, while an output thread retrieves and displays the accumulated readings.

## Overview

This application showcases:
- Creating message queues with `K_MSGQ_DEFINE()`
- Producer-consumer pattern using queues
- Lock-free inter-thread communication
- Sensor API usage (MCP9808 temperature sensor)
- Buffering data between threads with different timing

## Hardware Requirements

- ESP32-S3 DevKit-C (or compatible board)
- MCP9808 temperature sensor connected via I2C
- Proper devicetree configuration with `my_mcp9808` alias

## The Pattern: Producer-Consumer

This is a classic design pattern where:
- **Producer** (Sensor Thread): Generates data and puts it in the queue
- **Consumer** (Output Thread): Retrieves data from the queue and processes it
- **Queue**: Decouples the producers and consumers

```
Sensor Thread                Queue                Output Thread
(Producer)                 (Buffer)              (Consumer)
-------------              --------              --------------
Read sensor  ──────→     │ 25.1°C │
                         │ 25.2°C │
Read sensor  ──────→     │ 25.1°C │
                         │ 25.3°C │
Read sensor  ──────→     │ 25.2°C │           ←────── Get all
                         │ 25.1°C │           ←────── and print
Read sensor  ──────→     │ 25.4°C │
```

## How It Works

### Thread Architecture

The application has three execution contexts:

1. **Main Thread**
   - Priority: 0 (preemptive, highest)
   - Initializes sensor and threads
   - Idle loop

2. **Sensor Thread** (Producer)
   - Priority: 8 (preemptive, lower)
   - Options: 0 (no special options)
   - Start: `K_NO_WAIT` (immediately)
   - Arguments: `(void *)mcp` - passes sensor device handle
   - Reads temperature every 500ms
   - Puts readings in the queue
   - Handles queue full conditions

3. **Output Thread** (Consumer)
   - Priority: 7 (preemptive, higher priority than sensor)
   - Options: 0 (no special options)
   - Start: `K_NO_WAIT` (immediately)
   - Arguments: None (NULL, NULL, NULL)
   - Waits 5 seconds to let queue fill
   - Drains and prints all queued messages
   - Repeats

**Priority Hierarchy:** Output (7) > Sensor (8) means output thread can preempt sensor thread to ensure timely data processing

### Message Queue Definition

```c
K_MSGQ_DEFINE(my_queue,                    // Queue name
              sizeof(struct sensor_value), // Message size
              QUEUE_SIZE,                  // Max messages (20)
              4);                          // Alignment (4 bytes)
```

This creates a queue that can hold 20 temperature readings.

### Queue Operations

**Putting Data (Producer)**
```c
ret = k_msgq_put(&my_queue, &temperature, K_NO_WAIT);
if (ret < 0) {
    printk("Queue full\r\n");  // Queue is full, message dropped
}
```

**Getting Data (Consumer)**
```c
ret = k_msgq_get(&my_queue, &temperature, K_FOREVER);
if (ret == 0) {
    // Successfully got a message
}
```

**Checking Queue Status**
```c
int count = k_msgq_num_used_get(&my_queue);  // Number of messages in queue
```

## Code Flow

### Sensor Thread Flow

1. Cast device pointer from void* argument
2. Check if sensor device is ready
3. **Loop forever:**
   - Sleep 500ms
   - Fetch sample from sensor (`sensor_sample_fetch()`)
   - Get temperature value (`sensor_channel_get()`)
   - Put value in queue (`k_msgq_put()`)
   - Handle queue full condition (drop and report)

### Output Thread Flow

1. **Loop forever:**
   - Sleep 5 seconds (let queue accumulate data)
   - **While queue has messages:**
     - Get message from queue (`k_msgq_get()`)
     - Print temperature value
   - Repeat

### Sensor API Usage

```c
// Trigger sensor to take a reading
ret = sensor_sample_fetch(mcp);

// Retrieve the reading into a local variable
ret = sensor_channel_get(mcp, SENSOR_CHAN_AMBIENT_TEMP, &temperature);

// Temperature is stored in sensor_value struct
printk("Temperature: %d.%06d\n", temperature.val1, temperature.val2);
```

The `sensor_value` struct holds fixed-point values:
- `val1`: Integer part
- `val2`: Fractional part (microseconds resolution)

## Timing Analysis

### Data Collection Rate

- Sensor reads every **500ms** (2 Hz)
- Queue holds up to **20 messages**
- Output thread empties queue every **5000ms** (0.2 Hz)

### Queue Usage

In 5 seconds:
- Sensor puts: 5000ms ÷ 500ms = **10 messages**
- Output takes: **all 10 messages**

Queue won't fill unless output thread falls behind.

### What Happens if Queue Fills?

```c
ret = k_msgq_put(&my_queue, &temperature, K_NO_WAIT);
if (ret < 0) {
    printk("Queue full\r\n");  // Data is dropped
}
```

Options to handle full queue:
- `K_NO_WAIT`: Drop data (current implementation)
- `K_FOREVER`: Block until space available (can cause sensor to miss readings)
- `K_MSEC(100)`: Wait up to 100ms for space

## Building and Running

```bash
# Navigate to the project directory
cd apps/08_solution_sensor_queue

# Build for ESP32-S3
west build -b esp32s3_devkitc/esp32s3/procpu

# Flash to device
west flash

# Monitor output
west espressif monitor
```

## Expected Output

```
Starting sensor thread
Putting message on queue
Putting message on queue
Putting message on queue
...
Starting output thread
Putting message on queue
Temperature: 25.125000
Temperature: 25.187500
Temperature: 25.156250
Temperature: 25.218750
Temperature: 25.187500
Temperature: 25.156250
Temperature: 25.218750
Temperature: 25.187500
Temperature: 25.156250
Temperature: 25.218750
Putting message on queue
...
```

Every 5 seconds, you'll see a burst of temperature readings as the queue is drained.

## Key Concepts

### Message Queue vs. Mutex

| Feature | Message Queue | Mutex |
|---------|--------------|-------|
| Purpose | Data transfer | Mutual exclusion |
| Blocking | Producer or consumer | Only on lock |
| Data copy | Yes | No (protects shared memory) |
| Decoupling | High (buffering) | Low (shared access) |
| Use case | Different timing | Same data, different access times |

### When to Use Message Queues

✅ **Good for:**
- Producer and consumer run at different rates
- You need buffering between threads
- You want to decouple thread timing
- Passing data between threads

❌ **Not ideal for:**
- Large data structures (overhead of copying)
- Real-time requirements (copying latency)
- Simple flag or state sharing (use mutex instead)

### Lock-Free Communication

Message queues are lock-free from the user's perspective:
- No explicit mutex locking required
- Kernel handles synchronization internally
- Simpler code, less risk of deadlock

## Inter-Thread Communication Mechanisms in Zephyr

Zephyr provides multiple mechanisms for threads to communicate and synchronize. Each has different characteristics and use cases.

### 1. Message Queue (K_MSGQ) - Fixed-Size Messages

**What it is:** A FIFO buffer for passing fixed-size messages between threads.

**Characteristics:**
- Fixed message size (configured at compile time)
- FIFO ordering (first in, first out)
- Blocking/non-blocking put and get
- Message copying (sender and receiver have independent copies)
- Thread-safe (kernel handles synchronization)

**Definition:**
```c
// Define at compile time
K_MSGQ_DEFINE(my_queue,              // Queue name
              sizeof(struct data),   // Message size (bytes)
              10,                    // Maximum messages
              4);                    // Alignment (typically 4)

// Or initialize at runtime
struct k_msgq my_queue;
char queue_buffer[10 * sizeof(struct data)];
k_msgq_init(&my_queue, queue_buffer, sizeof(struct data), 10);
```

**API Operations:**
```c
// Put (send) - blocks if queue is full
int k_msgq_put(&my_queue, &data, timeout);
// Returns: 0 on success, -ENOMSG if full with K_NO_WAIT, -EAGAIN on timeout

// Get (receive) - blocks if queue is empty
int k_msgq_get(&my_queue, &data, timeout);
// Returns: 0 on success, -ENOMSG if empty with K_NO_WAIT, -EAGAIN on timeout

// Peek (read without removing)
int k_msgq_peek(&my_queue, &data);

// Get number of messages in queue
uint32_t count = k_msgq_num_used_get(&my_queue);

// Get free space in queue
uint32_t free = k_msgq_num_free_get(&my_queue);

// Purge (clear all messages)
k_msgq_purge(&my_queue);
```

**Example:**
```c
struct sensor_data {
    int32_t temperature;
    uint32_t timestamp;
};

K_MSGQ_DEFINE(sensor_queue, sizeof(struct sensor_data), 20, 4);

// Producer
void sensor_thread(void) {
    struct sensor_data data;
    while (1) {
        data.temperature = read_sensor();
        data.timestamp = k_uptime_get_32();
        
        if (k_msgq_put(&sensor_queue, &data, K_NO_WAIT) != 0) {
            printk("Queue full, data dropped\n");
        }
        k_msleep(100);
    }
}

// Consumer
void display_thread(void) {
    struct sensor_data data;
    while (1) {
        if (k_msgq_get(&sensor_queue, &data, K_FOREVER) == 0) {
            printk("Temp: %d at %u ms\n", data.temperature, data.timestamp);
        }
    }
}
```

**Best for:** Passing structured data, decoupling thread timing, buffering

---

### 2. FIFO Queue (K_FIFO) - Variable-Size Messages

**What it is:** A first-in-first-out queue for passing pointers to data structures.

**Characteristics:**
- Variable message size (passes pointers, not copies)
- Zero-copy (more efficient for large data)
- Requires memory management (caller manages allocation/deallocation)
- FIFO ordering
- Thread-safe

**Definition:**
```c
// Define at compile time
K_FIFO_DEFINE(my_fifo);

// Or initialize at runtime
struct k_fifo my_fifo;
k_fifo_init(&my_fifo);
```

**Data Structure Requirement:**
```c
// First member must be void* for linking
struct my_data {
    void *fifo_reserved;  // Reserved for kernel use (linking)
    int sensor_value;
    char name[32];
};
```

**API Operations:**
```c
// Put (send) - never blocks
k_fifo_put(&my_fifo, data_ptr);

// Get (receive) - blocks if empty
void *ptr = k_fifo_get(&my_fifo, timeout);

// Peek without removing
void *ptr = k_fifo_peek_head(&my_fifo);
void *ptr = k_fifo_peek_tail(&my_fifo);

// Check if empty
bool empty = k_fifo_is_empty(&my_fifo);

// Cancel waiting
k_fifo_cancel_wait(&my_fifo);
```

**Example:**
```c
struct sensor_data {
    void *fifo_reserved;  // Required!
    int temperature;
    int humidity;
};

K_FIFO_DEFINE(data_fifo);

// Producer
void sensor_thread(void) {
    while (1) {
        struct sensor_data *data = k_malloc(sizeof(struct sensor_data));
        if (data) {
            data->temperature = read_temp();
            data->humidity = read_humidity();
            k_fifo_put(&data_fifo, data);
        }
        k_msleep(1000);
    }
}

// Consumer
void process_thread(void) {
    while (1) {
        struct sensor_data *data = k_fifo_get(&data_fifo, K_FOREVER);
        printk("Temp: %d, Humidity: %d\n", data->temperature, data->humidity);
        k_free(data);  // Don't forget to free!
    }
}
```

**Best for:** Large data structures, zero-copy needed, variable-size messages

---

### 3. LIFO Queue (K_LIFO) - Stack-Based Queue

**What it is:** A last-in-first-out queue (stack) for passing pointers.

**Characteristics:**
- Same as FIFO but LIFO ordering (stack)
- Zero-copy (passes pointers)
- Most recent data processed first
- Thread-safe

**Definition:**
```c
K_LIFO_DEFINE(my_lifo);
```

**API Operations:**
```c
// Put (push)
k_lifo_put(&my_lifo, data_ptr);

// Get (pop)
void *ptr = k_lifo_get(&my_lifo, timeout);
```

**Example Use Case:**
```c
// Memory pool management (most recently freed blocks reused first)
K_LIFO_DEFINE(free_blocks);

void *alloc_block(void) {
    return k_lifo_get(&free_blocks, K_NO_WAIT);
}

void free_block(void *block) {
    k_lifo_put(&free_blocks, block);
}
```

**Best for:** Memory pools, undo operations, stack-like behavior

---

### 4. Mailbox (K_MBOX) - Synchronous Message Passing

**What it is:** Bidirectional, synchronous message passing with optional data exchange.

**Characteristics:**
- Synchronous exchange (sender and receiver rendezvous)
- Can exchange data or just signal
- Priority-based message delivery
- Both threads blocked until exchange completes
- More complex but powerful

**Definition:**
```c
K_MBOX_DEFINE(my_mbox);
```

**API Operations:**
```c
// Send message
struct k_mbox_msg send_msg;
send_msg.info = MSG_TYPE_SENSOR;
send_msg.size = sizeof(data);
send_msg.tx_data = &data;
send_msg.tx_target_thread = K_ANY;  // Any receiver

k_mbox_put(&my_mbox, &send_msg, K_FOREVER);

// Receive message
struct k_mbox_msg recv_msg;
recv_msg.size = sizeof(data);
recv_msg.rx_source_thread = K_ANY;  // Any sender

k_mbox_get(&my_mbox, &recv_msg, &data, K_FOREVER);
```

**Example:**
```c
#define MSG_TYPE_CMD    1
#define MSG_TYPE_DATA   2

K_MBOX_DEFINE(cmd_mbox);

// Sender
void control_thread(void) {
    struct k_mbox_msg msg;
    int command = START_MOTOR;
    
    msg.info = MSG_TYPE_CMD;
    msg.size = sizeof(command);
    msg.tx_data = &command;
    msg.tx_target_thread = K_ANY;
    
    k_mbox_put(&cmd_mbox, &msg, K_FOREVER);
    printk("Command sent and acknowledged\n");
}

// Receiver
void motor_thread(void) {
    struct k_mbox_msg msg;
    int command;
    
    msg.size = sizeof(command);
    msg.rx_source_thread = K_ANY;
    
    k_mbox_get(&cmd_mbox, &msg, &command, K_FOREVER);
    printk("Received command: %d\n", command);
    execute_command(command);
}
```

**Best for:** Request-response patterns, synchronous communication, RPC-like behavior

---

### 5. Pipe (K_PIPE) - Byte Stream

**What it is:** A circular buffer for passing arbitrary byte streams between threads.

**Characteristics:**
- Byte-oriented (not message-oriented)
- Can transfer partial data
- Circular buffer implementation
- No message boundaries
- Thread-safe

**Definition:**
```c
#define PIPE_SIZE 128
K_PIPE_DEFINE(my_pipe, PIPE_SIZE, 4);  // 128 bytes, 4-byte alignment
```

**API Operations:**
```c
// Put (write) bytes
size_t bytes_written;
k_pipe_put(&my_pipe, data, bytes_to_write, &bytes_written, 
           min_bytes, timeout);

// Get (read) bytes
size_t bytes_read;
k_pipe_get(&my_pipe, data, bytes_to_read, &bytes_read,
           min_bytes, timeout);

// Get available space/data
size_t avail = k_pipe_read_avail(&my_pipe);
size_t space = k_pipe_write_avail(&my_pipe);
```

**Example:**
```c
K_PIPE_DEFINE(uart_pipe, 256, 4);

// UART RX thread
void uart_rx_thread(void) {
    char buffer[32];
    while (1) {
        int len = uart_read(buffer, sizeof(buffer));
        size_t written;
        k_pipe_put(&uart_pipe, buffer, len, &written, len, K_FOREVER);
    }
}

// Parser thread
void parser_thread(void) {
    char byte;
    while (1) {
        size_t read;
        k_pipe_get(&uart_pipe, &byte, 1, &read, 1, K_FOREVER);
        parse_byte(byte);
    }
}
```

**Best for:** Streaming data, UART buffering, byte-oriented protocols

---

### 6. Work Queue (K_WORK_QUEUE) - Deferred Work

**What it is:** Execute work items (functions) in a dedicated thread context.

**Characteristics:**
- Offload work from ISR to thread
- Sequential execution of work items
- Can schedule delayed work
- Thread-safe submission

**Definition:**
```c
// Use system work queue
K_WORK_DEFINE(my_work, work_handler);

// Or define custom work queue
K_THREAD_STACK_DEFINE(my_wq_stack, 1024);
struct k_work_q my_work_q;

void init_work_queue(void) {
    k_work_queue_start(&my_work_q, my_wq_stack,
                       K_THREAD_STACK_SIZEOF(my_wq_stack),
                       5, NULL);  // Priority 5
}
```

**API Operations:**
```c
// Define work
void work_handler(struct k_work *work) {
    printk("Work executed in thread context\n");
}

K_WORK_DEFINE(my_work, work_handler);

// Submit work (from ISR or thread)
k_work_submit(&my_work);

// Delayed work
K_WORK_DELAYABLE_DEFINE(my_delayed_work, delayed_handler);
k_work_schedule(&my_delayed_work, K_MSEC(500));  // Run after 500ms
```

**Example:**
```c
struct sensor_work_data {
    struct k_work work;
    int sensor_value;
};

void sensor_work_handler(struct k_work *work) {
    struct sensor_work_data *data = 
        CONTAINER_OF(work, struct sensor_work_data, work);
    
    printk("Processing sensor value: %d\n", data->sensor_value);
    // Process in thread context
}

// From ISR
void sensor_isr(void) {
    static struct sensor_work_data work_data;
    work_data.sensor_value = read_sensor_register();
    
    k_work_init(&work_data.work, sensor_work_handler);
    k_work_submit(&work_data.work);
}
```

**Best for:** ISR to thread offload, deferred processing, timer callbacks

---

### 7. Event Objects (K_EVENT) - Multiple Event Signaling

**What it is:** A set of binary flags that threads can wait on (Zephyr 2.6+).

**Characteristics:**
- 32 independent event bits
- Wait for any or all events
- Non-consuming reads (events persist until cleared)
- Thread-safe

**Definition:**
```c
K_EVENT_DEFINE(my_events);
```

**API Operations:**
```c
// Set events (ISR-safe)
k_event_post(&my_events, 0x01);  // Set bit 0
k_event_post(&my_events, 0x06);  // Set bits 1 and 2

// Wait for any events
uint32_t events = k_event_wait(&my_events, 0x07, false, K_FOREVER);
// Waits for any of bits 0, 1, or 2

// Wait for all events
uint32_t events = k_event_wait(&my_events, 0x03, true, K_FOREVER);
// Waits for both bits 0 AND 1

// Clear events
k_event_clear(&my_events, 0xFF);  // Clear all

// Set all bits at once
k_event_set(&my_events, 0x0F);
```

**Example:**
```c
#define EVENT_SENSOR_READY    BIT(0)
#define EVENT_DATA_PROCESSED  BIT(1)
#define EVENT_ERROR          BIT(2)

K_EVENT_DEFINE(system_events);

// Sensor thread
void sensor_thread(void) {
    while (1) {
        read_sensor();
        k_event_post(&system_events, EVENT_SENSOR_READY);
        k_msleep(1000);
    }
}

// Main thread waits for events
void main_thread(void) {
    while (1) {
        uint32_t events = k_event_wait(&system_events,
                                       EVENT_SENSOR_READY | EVENT_ERROR,
                                       false,  // Any event
                                       K_FOREVER);
        
        if (events & EVENT_SENSOR_READY) {
            printk("Sensor data ready\n");
            process_data();
            k_event_clear(&system_events, EVENT_SENSOR_READY);
        }
        
        if (events & EVENT_ERROR) {
            printk("Error occurred\n");
            handle_error();
            k_event_clear(&system_events, EVENT_ERROR);
        }
    }
}
```

**Best for:** Multiple independent events, complex synchronization, state machines

---

### 8. Poll API - Wait on Multiple Objects

**What it is:** Wait on multiple synchronization objects simultaneously (like `select()` in Unix).

**Characteristics:**
- Monitor multiple objects (semaphores, FIFOs, signals, etc.)
- Wake up when any becomes ready
- Efficient multi-source waiting

**API Operations:**
```c
// Define poll events
K_POLL_EVENT_DEFINE(event1, K_POLL_TYPE_SEM_AVAILABLE, 
                    K_POLL_MODE_NOTIFY_ONLY, &my_sem);
K_POLL_EVENT_DEFINE(event2, K_POLL_TYPE_FIFO_DATA_AVAILABLE,
                    K_POLL_MODE_NOTIFY_ONLY, &my_fifo);

struct k_poll_event events[] = {event1, event2};

// Wait for any event
k_poll(events, 2, K_FOREVER);

// Check which event triggered
if (events[0].state == K_POLL_STATE_SEM_AVAILABLE) {
    k_sem_take(&my_sem, K_NO_WAIT);
}
if (events[1].state == K_POLL_STATE_FIFO_DATA_AVAILABLE) {
    data = k_fifo_get(&my_fifo, K_NO_WAIT);
}
```

**Best for:** Waiting on multiple sources, complex event handling

---

## Comparison of All Mechanisms

| Mechanism | Data Copy | Ordering | Size | ISR-Safe | Use Case |
|-----------|-----------|----------|------|----------|----------|
| **Message Queue** | Yes | FIFO | Fixed | Give only | Structured data, buffering |
| **FIFO** | No (pointers) | FIFO | Variable | Put only | Zero-copy, large data |
| **LIFO** | No (pointers) | LIFO | Variable | Put only | Memory pools, stacks |
| **Mailbox** | Optional | Priority | Any | No | Synchronous exchange, RPC |
| **Pipe** | Yes | Stream | Bytes | Both | Byte streams, UART |
| **Work Queue** | Context switch | FIFO | N/A | Submit only | Deferred work, ISR offload |
| **Event** | No | Flags | 32 bits | Post only | Multiple events, flags |
| **Poll** | No | N/A | N/A | No | Multiple sources |

## Decision Tree: Which Mechanism to Use?

```
START: Need inter-thread communication?
│
├─ Passing DATA between threads?
│  │
│  ├─ Fixed-size structures? → MESSAGE QUEUE
│  │  └─ Need buffering? → Yes, perfect
│  │
│  ├─ Variable-size or large data? → FIFO
│  │  └─ Need zero-copy? → Yes, perfect
│  │
│  ├─ Byte stream? → PIPE
│  │  └─ UART, serial data? → Yes, perfect
│  │
│  └─ Synchronous exchange? → MAILBOX
│     └─ Request-response? → Yes, perfect
│
├─ Just SIGNALING (no data)?
│  │
│  ├─ From ISR? → SEMAPHORE or EVENT
│  │  └─ Multiple events? → EVENT
│  │
│  ├─ Multiple independent signals? → EVENT (32 bits)
│  │
│  └─ Simple wake-up? → SEMAPHORE (binary)
│
├─ Offload work from ISR?
│  └─ → WORK QUEUE
│
└─ Wait on MULTIPLE sources?
   └─ → POLL API
```

## Performance Considerations

### Memory Usage (Approximate)

```c
// Message Queue: overhead + (msg_size × count)
K_MSGQ_DEFINE(q, 16, 10, 4);  // ~16 + (16×10) = ~176 bytes

// FIFO/LIFO: minimal overhead (~16-20 bytes)
K_FIFO_DEFINE(f);  // ~20 bytes + your data structs

// Pipe: overhead + buffer size
K_PIPE_DEFINE(p, 256, 4);  // ~16 + 256 = ~272 bytes

// Event: minimal
K_EVENT_DEFINE(e);  // ~12-16 bytes
```

### CPU Overhead (Relative)

- **Fastest**: Event, Semaphore (just flag manipulation)
- **Fast**: FIFO/LIFO (pointer manipulation)
- **Medium**: Message Queue (memory copy)
- **Slower**: Pipe (byte-by-byte, circular buffer logic)
- **Slowest**: Mailbox (synchronization overhead)

## Learning Objectives

After completing this example, you should understand:
1. Producer-consumer design pattern
2. How to create and use message queues in Zephyr
3. Lock-free inter-thread communication
4. Buffering data between threads with different timing
5. Sensor API usage (fetch and get pattern)
6. Handling queue full/empty conditions
7. Fixed-point number representation (sensor_value)

## Common Patterns

### Multiple Producers

```c
// Multiple threads can put to the same queue
k_msgq_put(&my_queue, &data1, K_NO_WAIT);  // Thread 1
k_msgq_put(&my_queue, &data2, K_NO_WAIT);  // Thread 2
```

### Multiple Consumers

```c
// Multiple threads can get from the same queue
k_msgq_get(&my_queue, &data1, K_FOREVER);  // Thread 1 (blocks)
k_msgq_get(&my_queue, &data2, K_FOREVER);  // Thread 2 (blocks)
```

### Purge Queue

```c
k_msgq_purge(&my_queue);  // Remove all messages
```

### Peek (Without Removing)

```c
k_msgq_peek(&my_queue, &temperature);  // Read without consuming
```

## Exercises

1. **Change Queue Size**: Set `QUEUE_SIZE` to 5 and observe what happens
2. **Change Timing**: Make sensor faster than output and observe queue filling
3. **Add Priority**: Implement a high-priority message type
4. **Multiple Sensors**: Create multiple sensor threads writing to the same queue
5. **Ringbuffer Alternative**: Compare with `struct k_msgq` vs. custom ringbuffer
6. **Add Statistics**: Track dropped messages, queue high-water mark

## Debugging Tips

- Use `k_msgq_num_used_get()` to monitor queue usage
- Print messages when queue is full to detect backpressure
- Check `sensor_sample_fetch()` return values for sensor errors
- Enable sensor driver debugging: `CONFIG_SENSOR_LOG_LEVEL_DBG=y`

## Performance Considerations

### Message Size Impact

- Current: `sizeof(struct sensor_value)` = 8 bytes
- Total queue memory: 8 bytes × 20 messages = 160 bytes
- Larger messages = more copying overhead

### Zero-Copy Alternative

For large data, use pointers:
```c
K_MSGQ_DEFINE(ptr_queue, sizeof(void*), QUEUE_SIZE, 4);

// Put pointer (not data)
k_msgq_put(&ptr_queue, &data_ptr, K_NO_WAIT);

// Remember to manage memory lifetime!
```

## Next Steps

- See `08_demo_multithreading` for basic threading concepts
- See `08_demo_mutex` for shared memory synchronization
- Learn about FIFOs for variable-sized messages
- Explore semaphores for event signaling

## References

- [Zephyr Message Queue Documentation](https://docs.zephyrproject.org/latest/kernel/services/data_passing/message_queues.html)
- [Zephyr Sensor API Documentation](https://docs.zephyrproject.org/latest/hardware/peripherals/sensor.html)
- [Data Passing Services](https://docs.zephyrproject.org/latest/kernel/services/data_passing/index.html)

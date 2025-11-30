# Building a Driver in Zephyr RTOS

Based on the Nintendo Nunchuk I2C driver example, here's a comprehensive guide to building a driver in Zephyr:

## 1. **Define the Driver API Header**
Create a public header file in `include/drivers/` that defines:

### Data Structures
```c
struct device_data {
    /* Public data structure for sensor readings */
    uint8_t value_x;
    uint16_t value_y;
    bool status;
    uint32_t timestamp;
};
```

### Enumerations and Types
```c
enum device_modes {
    MODE_NORMAL,
    MODE_ADVANCED,
};

/* Callback function types */
typedef void (*data_callback_t)(const struct device *dev, 
                                const struct device_data *data, 
                                void *user_data);
```

### Public API Functions
```c
int device_read(const struct device *dev, struct device_data *data);
int device_set_callback(const struct device *dev, data_callback_t callback, void *user_data);
int device_calibrate(const struct device *dev);
```

## 2. **Create Device Tree Binding**
Define a YAML binding file in `dts/bindings/` (e.g., `vendor,device-name.yaml`):

```yaml
compatible: "vendor,device-name"
include: [base.yaml, i2c-device.yaml]

properties:
  poll-interval-ms:
    type: int
    default: 50
    description: "Polling interval in milliseconds"
  
  deadzone:
    type: int
    default: 10
    description: "Joystick deadzone threshold"
  
  enable-features:
    type: boolean
    description: "Enable advanced features"
  
  device-type:
    type: string
    enum: ["type1", "type2"]
    default: "type1"
```

## 3. **Implement the Driver Source**

### Driver Structure Pattern
```c
#define DT_DRV_COMPAT vendor_device_name

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(device_name, CONFIG_DEVICE_LOG_LEVEL);
```

### Configuration Structure (from Device Tree)
```c
struct device_config {
    struct i2c_dt_spec i2c;        /* I2C configuration */
    uint32_t poll_interval_ms;      /* From DT properties */
    uint8_t deadzone;
    bool enable_features;
    const char *device_type;
};
```

### Runtime Data Structure (Private)
```c
struct device_data_private {
    /* Work queue for polling */
    struct k_work_delayable work;
    
    /* CRITICAL: Store device pointer for work handler */
    const struct device *device;
    
    /* Cached sensor data */
    struct device_data sensor_data;
    
    /* Callbacks */
    data_callback_t callback;
    void *callback_user_data;
    
    /* Synchronization */
    struct k_mutex data_mutex;
    
    /* Driver state */
    bool initialized;
};
```

## 4. **Implement Core Driver Functions**

### Hardware Initialization
```c
static int device_hw_init(const struct device *dev)
{
    const struct device_config *cfg = dev->config;
    uint8_t cmd_data[2];
    int ret;
    
    /* Send initialization commands */
    cmd_data[0] = INIT_CMD;
    cmd_data[1] = INIT_VAL;
    ret = i2c_write_dt(&cfg->i2c, cmd_data, 2);
    if (ret < 0) {
        LOG_ERR("Init failed: %d", ret);
        return ret;
    }
    
    k_msleep(INIT_DELAY_MS);
    return 0;
}
```

### Work Handler for Polling
```c
static void device_work_handler(struct k_work *work)
{
    /* Convert work to delayable work */
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    
    /* Get private data from work */
    struct device_data_private *priv = CONTAINER_OF(dwork,
                                                    struct device_data_private,
                                                    work);
    
    /* CRITICAL: Get device from stored pointer */
    const struct device *dev = priv->device;
    const struct device_config *cfg = dev->config;
    
    /* Read sensor data */
    struct device_data new_data;
    int ret = read_sensor_data(dev, &new_data);
    
    /* Update cached data with mutex protection */
    k_mutex_lock(&priv->data_mutex, K_FOREVER);
    priv->sensor_data = new_data;
    k_mutex_unlock(&priv->data_mutex);
    
    /* Call callback if registered */
    if (priv->callback) {
        priv->callback(dev, &new_data, priv->callback_user_data);
    }
    
    /* Schedule next reading */
    k_work_schedule(&priv->work, K_MSEC(cfg->poll_interval_ms));
}
```

### Driver Initialization
```c
static int driver_init(const struct device *dev)
{
    const struct device_config *cfg = dev->config;
    struct device_data_private *priv = dev->data;
    int ret;
    
    LOG_INF("Initializing driver");
    
    /* CRITICAL: Store device pointer */
    priv->device = dev;
    
    /* Verify bus is ready */
    if (!i2c_is_ready_dt(&cfg->i2c)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }
    
    /* Initialize synchronization */
    ret = k_mutex_init(&priv->data_mutex);
    if (ret < 0) {
        return ret;
    }
    
    /* Initialize work handler */
    k_work_init_delayable(&priv->work, device_work_handler);
    
    /* Initialize hardware */
    ret = device_hw_init(dev);
    if (ret < 0) {
        return ret;
    }
    
    priv->initialized = true;
    
    /* Start periodic polling */
    k_work_schedule(&priv->work, K_MSEC(cfg->poll_interval_ms));
    
    LOG_INF("Driver ready");
    return 0;
}
```

## 5. **Device Instantiation Macro**
```c
#define DEVICE_INIT(inst) \
    static const struct device_config device_config_##inst = { \
        .i2c = I2C_DT_SPEC_INST_GET(inst), \
        .poll_interval_ms = DT_INST_PROP(inst, poll_interval_ms), \
        .deadzone = DT_INST_PROP(inst, deadzone), \
        .enable_features = DT_INST_PROP(inst, enable_features), \
        .device_type = DT_INST_PROP(inst, device_type), \
    }; \
    static struct device_data_private device_data_##inst; \
    DEVICE_DT_INST_DEFINE(inst, driver_init, NULL, \
                          &device_data_##inst, &device_config_##inst, \
                          POST_KERNEL, CONFIG_DEVICE_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(DEVICE_INIT)
```

## 6. **Kconfig Configuration**
Create `drivers/sensor/device_name/Kconfig`:

```kconfig
config DEVICE_NAME
    bool "Device Name driver"
    default y
    depends on DT_HAS_VENDOR_DEVICE_NAME_ENABLED
    select I2C
    help
      Enable driver for Device Name sensor.

if DEVICE_NAME

config DEVICE_NAME_LOG_LEVEL
    int "Log level"
    default 3
    range 0 4
    help
      Log level for Device Name driver.

config DEVICE_NAME_INIT_PRIORITY
    int "Init priority"
    default 70
    help
      Device initialization priority.

config DEVICE_NAME_ADVANCED_FEATURES
    bool "Enable advanced features"
    default n
    help
      Enable advanced feature detection.

endif # DEVICE_NAME
```

## 7. **CMakeLists.txt**
```cmake
zephyr_library()
zephyr_library_sources_ifdef(CONFIG_DEVICE_NAME device_name.c)
zephyr_library_sources_ifdef(CONFIG_DEVICE_NAME_ADVANCED advanced.c)
```

## 8. **Device Tree Usage**
In your board's device tree overlay:

```dts
&i2c0 {
    device_name: device@52 {
        compatible = "vendor,device-name";
        reg = <0x52>;
        poll-interval-ms = <20>;
        deadzone = <15>;
        enable-features;
        device-type = "type2";
    };
};
```

## Key Design Patterns

### 1. **Work Queue Pattern**
Use for periodic polling or deferred processing:
- Initialize with `k_work_init_delayable()`
- Store device pointer in private data
- Use `CONTAINER_OF` to recover context
- Schedule with `k_work_schedule()`

### 2. **Mutex Protection**
Protect shared data between ISR/work handlers and API calls:
```c
k_mutex_lock(&priv->data_mutex, K_FOREVER);
/* Critical section */
k_mutex_unlock(&priv->data_mutex);
```

### 3. **Callback Pattern**
Allow applications to register for asynchronous notifications:
- Define callback typedef in API header
- Store callback pointer and user data
- Call from work handler or ISR

### 4. **Configuration vs Runtime Data**
- **Config struct**: Read-only, from device tree
- **Data struct**: Runtime state, mutable

### 5. **Error Handling**
- Check return values from all I2C operations
- Log errors with appropriate levels
- Gracefully handle initialization failures

## Build System Integration

1. Add driver path to `drivers/sensor/CMakeLists.txt`
2. Add Kconfig source to `drivers/sensor/Kconfig`
3. Enable in project configuration: `CONFIG_DEVICE_NAME=y`

## Testing Checklist

- [ ] Device initializes without I2C device connected (fails gracefully)
- [ ] Device initializes with I2C device connected
- [ ] Polling works at configured interval
- [ ] Callbacks fire correctly
- [ ] Mutex protection prevents data corruption
- [ ] Multiple instances work (if supported)
- [ ] Power management transitions (if implemented)
- [ ] All Kconfig options work as expected

This structure provides a robust, maintainable driver that integrates seamlessly with Zephyr's device model and follows best practices for embedded driver development.

-------

# Deep Dive into DEVICE_DT_INST_DEFINE

The `DEVICE_DT_INST_DEFINE` macro is the heart of Zephyr's device driver instantiation system. Let's break it down completely:

## Macro Signature and Purpose

```c
DEVICE_DT_INST_DEFINE(inst, init_fn, pm, data, config, level, prio, api)
```

This macro creates a complete device instance at compile time, registering it with Zephyr's device management system.

## Parameter-by-Parameter Analysis

### 1. **`inst` - Instance Number**
```c
inst
```
- This is the device tree instance number (0, 1, 2, ...)
- Comes from device tree nodes with matching `compatible` property
- Used with `DT_DRV_COMPAT` to identify which nodes to instantiate

**Example**: If your device tree has:
```dts
&i2c0 {
    nunchuk0: nunchuk@52 {  /* inst = 0 */
        compatible = "nintendo,nunchuk-extended";
        reg = <0x52>;
    };
    nunchuk1: nunchuk@53 {  /* inst = 1 */
        compatible = "nintendo,nunchuk-extended";
        reg = <0x53>;
    };
};
```

### 2. **`driver_init` - Initialization Function**
```c
driver_init
```
- Function called during system initialization
- Signature: `int init_fn(const struct device *dev)`
- Must return 0 on success, negative errno on failure
- Called automatically based on initialization level and priority

**Example**:
```c
static int nunchuk_init(const struct device *dev)
{
    /* Hardware initialization */
    /* Set up work queues */
    /* Initialize mutexes */
    return 0;
}
```

### 3. **`NULL` - Power Management**
```c
NULL
```
- Power management action callback
- Set to NULL if no PM support
- If provided, signature: `int pm_action(const struct device *dev, enum pm_device_action action)`

**With PM support**:
```c
static int device_pm_action(const struct device *dev, enum pm_device_action action)
{
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        /* Put device in low power */
        break;
    case PM_DEVICE_ACTION_RESUME:
        /* Wake device up */
        break;
    default:
        return -ENOTSUP;
    }
    return 0;
}
/* Then use: DEVICE_DT_INST_DEFINE(..., device_pm_action, ...) */
```

### 4. **`&device_data_##inst` - Runtime Data**
```c
&device_data_##inst
```
- Pointer to device's mutable runtime data structure
- `##inst` concatenates the instance number (e.g., `device_data_0`)
- Accessible via `dev->data` in driver code
- Contains state, buffers, work items, etc.

**Token Pasting Example**:
```c
/* If inst = 0, this becomes: &device_data_0 */
/* If inst = 1, this becomes: &device_data_1 */

static struct nunchuk_data_private device_data_0;  /* Created by macro */
static struct nunchuk_data_private device_data_1;  /* Created by macro */
```

### 5. **`&device_config_##inst` - Configuration Data**
```c
&device_config_##inst
```
- Pointer to device's read-only configuration
- Contains device tree properties and hardware config
- Accessible via `dev->config` in driver code
- Typically marked `const` for ROM storage

**Example Structure**:
```c
static const struct nunchuk_config device_config_0 = {
    .i2c = {
        .bus = DEVICE_DT_GET(DT_INST_BUS(0)),
        .addr = DT_INST_REG_ADDR(0),
    },
    .poll_interval_ms = 50,  /* From device tree */
    .deadzone = 10,           /* From device tree */
};
```

### 6. **`POST_KERNEL` - Initialization Level**
```c
POST_KERNEL
```
Defines when during boot the device initializes:

| Level | Value | Description | Use For |
|-------|-------|-------------|---------|
| `EARLY` | 0 | Very early init | Critical hardware |
| `PRE_KERNEL_1` | 10 | Before kernel services | Essential drivers |
| `PRE_KERNEL_2` | 20 | Before kernel, after PRE_KERNEL_1 | Bus controllers |
| `POST_KERNEL` | 30 | After kernel services available | Most drivers |
| `APPLICATION` | 40 | Application level | User devices |
| `SMP` | 50 | After SMP initialization | Multi-core dependent |

**Why POST_KERNEL?**
- Kernel services (threads, mutexes, work queues) are available
- I2C/SPI buses are initialized (they're usually PRE_KERNEL_2)
- Safe for most sensor/peripheral drivers

### 7. **`CONFIG_DEVICE_INIT_PRIORITY` - Priority Within Level**
```c
CONFIG_DEVICE_INIT_PRIORITY
```
- Fine-grained ordering within the initialization level
- Range: 0-99 (lower numbers = higher priority = earlier init)
- Usually defined in Kconfig

**Example Priorities**:
```kconfig
config NUNCHUK_INIT_PRIORITY
    int "Init priority"
    default 70
    range 0 99
    help
      Nunchuk initialization priority within POST_KERNEL level.
      Lower numbers initialize earlier.
```

**Common Priority Patterns**:
- 0-9: Critical dependencies
- 10-49: Bus drivers, essential services  
- 50-79: Standard drivers (sensors, peripherals)
- 80-99: Optional/dependent drivers

### 8. **`NULL` - API Structure**
```c
NULL
```
- Pointer to driver API function table
- Used for subsystem APIs (like sensor API, flash API)
- NULL for custom drivers without standardized API

**With API Structure**:
```c
static const struct sensor_driver_api nunchuk_api = {
    .sample_fetch = nunchuk_sample_fetch,
    .channel_get = nunchuk_channel_get,
    .attr_set = nunchuk_attr_set,
};
/* Then use: DEVICE_DT_INST_DEFINE(..., &nunchuk_api) */
```

## Complete Expansion Example

When the macro expands for instance 0, it creates something like:

```c
/* Original macro call */
DEVICE_DT_INST_DEFINE(0, nunchuk_init, NULL,
                      &nunchuk_data_0, &nunchuk_config_0,
                      POST_KERNEL, CONFIG_NUNCHUK_INIT_PRIORITY, NULL);

/* Expands to (simplified): */
static const struct device __device_dts_ord_172 __used __attribute__((__section__(".z_device_POST_KERNEL_070"))) = {
    .name = "nunchuk@52",
    .config = &nunchuk_config_0,
    .api = NULL,
    .state = NULL,
    .data = &nunchuk_data_0,
    .init = nunchuk_init,
    .pm = NULL,
    /* ... other fields ... */
};

/* Also creates init infrastructure: */
static const struct init_entry __init_dts_ord_172 __used __attribute__((__section__(".z_init_POST_KERNEL_070"))) = {
    .init = device_init_wrapper,
    .dev = &__device_dts_ord_172,
};
```

## The Magic of DT_INST_FOREACH_STATUS_OKAY

The complete instantiation pattern:

```c
#define NUNCHUK_INIT(inst) \
    static const struct nunchuk_config nunchuk_config_##inst = { \
        .i2c = I2C_DT_SPEC_INST_GET(inst), \
        .poll_interval_ms = DT_INST_PROP(inst, poll_interval_ms), \
    }; \
    static struct nunchuk_data_private nunchuk_data_##inst; \
    DEVICE_DT_INST_DEFINE(inst, nunchuk_init, NULL, \
                          &nunchuk_data_##inst, &nunchuk_config_##inst, \
                          POST_KERNEL, CONFIG_NUNCHUK_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(NUNCHUK_INIT)
```

This expands to call `NUNCHUK_INIT(n)` for each instance where:
- The compatible matches `DT_DRV_COMPAT`
- The status is "okay" in device tree

## Initialization Flow

```
System Boot
    ↓
PRE_KERNEL_1 devices init (priority order)
    ↓
PRE_KERNEL_2 devices init (priority order)
    ↓
Kernel services start
    ↓
POST_KERNEL devices init (priority order)  ← Your driver here
    ├─ Priority 0-49 devices
    ├─ Priority 50 devices
    ├─ Priority 70 devices (CONFIG_NUNCHUK_INIT_PRIORITY)
    │   └─ nunchuk_init() called for each instance
    └─ Priority 71-99 devices
    ↓
APPLICATION devices init
    ↓
main() starts
```

## Accessing the Device Later

After initialization, the device can be accessed:

```c
/* By label from device tree */
const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(nunchuk0));

/* By compatible and instance */
const struct device *dev = DEVICE_DT_INST_GET(0);

/* Check if ready */
if (device_is_ready(dev)) {
    nunchuk_read(dev, &data);
}
```

## Key Points to Remember

1. **One macro call = One device instance** - Each creates a complete device structure
2. **Compile-time registration** - No runtime registration needed
3. **Automatic initialization** - Called in order during boot
4. **Static allocation** - Everything allocated at compile time, no heap
5. **Device tree driven** - Number of instances determined by DT
6. **Type safety** - config and data pointers are type-checked at compile time

This macro is the cornerstone of Zephyr's efficient, deterministic device model, enabling static initialization without runtime overhead while maintaining flexibility through device tree configuration.
// ./Kconfig

/*
choice TARGET_PLATFORM
   bool "Espressif target platform choose"
   default IDF_TARGET_ESP8266
   help
       Choose the specific target platform which you will use.

config IDF_TARGET_ESP32
   bool "esp32"
config IDF_TARGET_ESP8266
   bool "esp8266"
endchoice
*/
#define CONFIG_IDF_TARGET_ESP8266

// -----------------------------------------------------------------------------

// ./components/bootloader/Kconfig.projbuild

/*
config BOOTLOADER_INIT_SPI_FLASH
    bool "Bootloader init SPI flash"
    default y
    help
        Enable this option, software will initialize SPI flash clock and I/O mode at bootloader instead of at APP.
        So it will speed up system starting and reduce the time cost at loading firmware.

        If your system bootloader is based on v3.0, the option must not be enable, because the v3.0 bootloader don't support
        this function.
*/
//#define CONFIG_BOOTLOADER_INIT_SPI_FLASH

/*
choice LOG_BOOTLOADER_LEVEL
   bool "Bootloader log verbosity"
   default LOG_BOOTLOADER_LEVEL_INFO
   help
       Specify how much output to see in bootloader logs.

config LOG_BOOTLOADER_LEVEL_NONE
   bool "No output"
config LOG_BOOTLOADER_LEVEL_ERROR
   bool "Error"
config LOG_BOOTLOADER_LEVEL_WARN
   bool "Warning"
config LOG_BOOTLOADER_LEVEL_INFO
   bool "Info"
config LOG_BOOTLOADER_LEVEL_DEBUG
   bool "Debug"
config LOG_BOOTLOADER_LEVEL_VERBOSE
   bool "Verbose"
endchoice

config LOG_BOOTLOADER_LEVEL
    int
    default 0 if LOG_BOOTLOADER_LEVEL_NONE
    default 1 if LOG_BOOTLOADER_LEVEL_ERROR
    default 2 if LOG_BOOTLOADER_LEVEL_WARN
    default 3 if LOG_BOOTLOADER_LEVEL_INFO
    default 4 if LOG_BOOTLOADER_LEVEL_DEBUG
    default 5 if LOG_BOOTLOADER_LEVEL_VERBOSE
*/
#define CONFIG_LOG_BOOTLOADER_LEVEL 0

/*
config BOOTLOADER_CHECK_APP_SUM
    bool "Check APP binary data sum before loading"
    default y
    help
        If enable this option, bootloader will check the sum of app binary data before load it to run.
*/
//#define CONFIG_ENABLE_BOOT_CHECK_SUM 1

/*
config BOOTLOADER_CHECK_APP_HASH
    bool "Check APP binary data hash before loading"
    default n
    help
        If enable this option, bootloader will check the hash of app binary data before load it to run.
*/
//#define CONFIG_ENABLE_BOOT_CHECK_SHA256 1

/*
config BOOTLOADER_APP_TEST
    bool "GPIO triggers boot from test app partition"
    default n
    help
        Allows to run the test app from "TEST" partition.
        A boot from "test" partition will occur if there is a GPIO input pulled low while device starts up.
        See settings below.
*/
//#define CONFIG_BOOTLOADER_APP_TEST

/*
config BOOTLOADER_APP_TEST_IN_OTA_1
    depends on BOOTLOADER_APP_TEST && IDF_TARGET_ESP8266
    bool "Put test app in the ota_1 partition"
    default y
    help
        For the small SPI Flash solution, there maybe no enough space for the test app partition.
        By enable this option, test app will locate in ota_1 partition by default.
        After ota, the test app will be erased and re-write as new app.

        If you disable this, make sure there has a test app partition in you partition table CVS.
*/
//#define CONFIG_BOOTLOADER_APP_TEST_IN_OTA_1

// -----------------------------------------------------------------------------

// ./components/esp8266/Kconfig

/*
choice ESP8266_DEFAULT_CPU_FREQ_MHZ
    prompt "CPU frequency"
    default ESP8266_DEFAULT_CPU_FREQ_80
    help
        CPU frequency to be set on application startup.

    config ESP8266_DEFAULT_CPU_FREQ_80
        bool "80 MHz"
    config ESP8266_DEFAULT_CPU_FREQ_160
        bool "160 MHz"
endchoice

config ESP8266_DEFAULT_CPU_FREQ_MHZ
    int
    default 80 if ESP8266_DEFAULT_CPU_FREQ_80
    default 160 if ESP8266_DEFAULT_CPU_FREQ_160
*/
#define CONFIG_ESP8266_DEFAULT_CPU_FREQ_160

/*
choice NEWLIB_STDOUT_LINE_ENDING
    prompt "Line ending for UART output"
    default NEWLIB_STDOUT_LINE_ENDING_CRLF
    help
        This option allows configuring the desired line endings sent to UART
        when a newline ('\n', LF) appears on stdout.
        Three options are possible:

        CRLF: whenever LF is encountered, prepend it with CR

        LF: no modification is applied, stdout is sent as is

        CR: each occurence of LF is replaced with CR

        This option doesn't affect behavior of the UART driver (drivers/uart.h).

config NEWLIB_STDOUT_LINE_ENDING_CRLF
    bool "CRLF"
config NEWLIB_STDOUT_LINE_ENDING_LF
    bool "LF"
config NEWLIB_STDOUT_LINE_ENDING_CR
    bool "CR"
endchoice
*/
#define CONFIG_NEWLIB_STDOUT_LINE_ENDING_CRLF

/*
choice ESP_FILENAME_MACRO
    prompt "File name macro text"
    default ESP_FILENAME_MACRO_NO_PATH
    help
        This option allows configuring the macro __ESP_FILE__ text.
        Three options are possible:

        no PATH: strip the path of macro __FILE__, for example: __FILE__="a/b/c/d.h", then __ESP_FILE__="d.h"

        raw: same as macro __FILE__

        null: text is string "null"

config ESP_FILENAME_MACRO_NO_PATH
    bool "no PATH"
config ESP_FILENAME_MACRO_RAW
    bool "raw"
config ESP_FILENAME_MACRO_NULL
    bool "null"
endchoice
*/
// In makefile: __ESP_FILE__=__FILE__

/*
config USING_NEW_ETS_VPRINTF
    bool "Using new ets_vprintf instead of rom code"
    default y
    help
        Enable this option, SDK will use new "ets_vprintf" function instead of old code "ets_vprintf" which is depend on ROM
        code "ets_io_vprintf".

        Note: Bootloader can't use this function.
*/
#define CONFIG_USING_NEW_ETS_VPRINTF

/*
config SOC_FULL_ICACHE
    bool "Enable full cache mode"
    default n
    help
        Enable this option, full 32 KB iram instead of 16 KB iram will be used as icache, so the heap use can use
        may reduce a lot.
*/
//#define CONFIG_SOC_FULL_ICACHE

/*
choice CONSOLE_UART_NUM
    prompt "UART peripheral to use for console output (0-1)"
    depends on CONSOLE_UART_CUSTOM
    default CONSOLE_UART_CUSTOM_NUM_0
    help
        Configrate output console UART for "ets_printf", "printf", "ESP_LOGX" and so on.

config CONSOLE_UART_CUSTOM_NUM_0
    bool "UART0"
config CONSOLE_UART_CUSTOM_NUM_1
    bool "UART1"
endchoice

config CONSOLE_UART_NUM
    int
    default 0 if CONSOLE_UART_DEFAULT || CONSOLE_UART_NONE
    default 0 if CONSOLE_UART_CUSTOM_NUM_0
    default 1 if CONSOLE_UART_CUSTOM_NUM_1
*/
#define CONFIG_CONSOLE_UART_NUM 0

/*
config CONSOLE_UART_BAUDRATE
    int "UART console baud rate"
    depends on CONSOLE_UART_DEFAULT || CONSOLE_UART_CUSTOM
    default 74880
    range 1200 4000000
*/
#define CONFIG_CONSOLE_UART_BAUDRATE 74880

/*
config UART0_SWAP_IO
    bool "Swap UART0 I/O pins"
    default n
    help
        Enable this option, UART0's I/O pins are swaped: TXD <-> RTS, RTX <-> CTS.
*/
#define UART0_SWAP_IO 0

/*
config MAIN_TASK_STACK_SIZE
    int "Main task stack size"
    default 3584
    help
        Configure the "main task" stack size. This is the stack of the task
        which calls app_main(). If app_main() returns then this task is deleted
        and its stack memory is freed.
*/
#define CONFIG_MAIN_TASK_STACK_SIZE 3584

/*
config TASK_WDT
    bool "Initialize Task Watchdog Timer on startup"
    default y
    help
        The Task Watchdog Timer can be used to make sure individual tasks are still
        running. Enabling this option will cause the Task Watchdog Timer to be
        initialized automatically at startup. The Task Watchdog timer can be 
        initialized after startup as well.
*/
#define CONFIG_TASK_WDT

/*
config TASK_WDT_PANIC
    bool "Invoke panic handler on Task Watchdog timeout"
    default y
    help
        If this option is enabled, the Task Watchdog Timer will be configured to
        trigger the panic handler when it times out. And it may cost some time.
*/
#define CONFIG_TASK_WDT_PANIC

/*
choice TASK_WDT_TIMEOUT_S
    prompt "Task Watchdog timeout period (seconds)"
    default TASK_WDT_TIMEOUT_15N
    help
        Timeout period configuration for the Task Watchdog Timer in seconds.
        This is also configurable at run time.

config TASK_WDT_TIMEOUT_13N
    bool "6.5536s"
config TASK_WDT_TIMEOUT_14N
    bool "13.1072s"
config TASK_WDT_TIMEOUT_15N
    bool "26.2144s"
endchoice

config TASK_WDT_TIMEOUT_S
    int
    default 13 if TASK_WDT_TIMEOUT_13N
    default 14 if TASK_WDT_TIMEOUT_14N
    default 15 if TASK_WDT_TIMEOUT_15N
*/
#define CONFIG_TASK_WDT_TIMEOUT_S 15

/*
config RESET_REASON
    bool "Enable reset reason"
    default y
    help
        Enable this option, the reset reason function can be used, or compiler will show function linking error.
*/
#define CONFIG_RESET_REASON 1

/*
config WIFI_PPT_TASKSTACK_SIZE
    int "ppT task stack size"
    default 2048
    range 2048 8192
    help
        Configure the "ppT task" stack size. This is the stack of the task
        which calls promiscuous callback function. So if user's function is
        complex, the stack must be set larger.
*/
#define CONFIG_WIFI_PPT_TASKSTACK_SIZE 2048

/*
config EVENT_LOOP_STACK_SIZE
    int "Event loop stack size"
    default 2048
    help
        Configure the Event loop task stack size per application.
*/
#define CONFIG_EVENT_LOOP_STACK_SIZE 2048

/*
choice CRYSTAL_USED
    prompt "Crystal used in your module or board"
    default CRYSTAL_USED_26MHZ
    help
        For most modules, 26MHz is the default crystal. If you use special module,
        you can reconfigure this option.

config CRYSTAL_USED_26MHZ
    bool "26MHz"
config CRYSTAL_USED_40MHZ
    bool "40MHz"
endchoice
*/
#define CONFIG_CRYSTAL_USED_26MHZ 1

/*
config INIT_OS_BEFORE_START
    bool "Init OS before starting it"
    default n
    depends on DISABLE_FREERTOS
    help
        Some OSes should initialize their global data structure before starting them. rt-thread is like this one.

        FreeRTOS need not do this.
*/
//#define CONFIG_INIT_OS_BEFORE_START

/*
config SCAN_AP_MAX
    int "Max scan AP number"
    range 1 64
    default 32
    help
        Function "esp_wifi_scan_get_ap_num" return value will be less than this. It is said that user cannot
        scan more than this.

        User can use own function "esp_wifi_scan_get_ap_num_max" to determin how many AP to scan , too.
*/
#define CONFIG_SCAN_AP_MAX 32

/*
config WIFI_TX_RATE_SEQUENCE_FROM_HIGH
    bool "Set wifi tx rate from 54M to 1M"
    default y
    help
        If this option is enabled, Wifi will try to send packets first from high rate(54M). If it fails, it will 
        try at low rate until the transmission is successful.
*/
#define CONFIG_WIFI_TX_RATE_SEQUENCE_FROM_HIGH 1

/*
config ESP8266_WIFI_DEBUG_LOG_ENABLE
    bool "Enable WiFi debug log"
    default n
    help
        Select this option to enable WiFi debug log
*/
#define CONFIG_ESP8266_WIFI_DEBUG_LOG_ENABLE 0

/*
choice ESP8266_WIFI_DEBUG_LOG_LEVEL
    depends on ESP8266_WIFI_DEBUG_LOG_ENABLE
    prompt "The DEBUG level is enabled"
    default ESP8266_WIFI_DEBUG_LOG_ERROR
    help
        The WiFi log is divided into the following levels: ERROR,WARNING,INFO,DEBUG,VERBOSE.
        The ERROR level is enabled by default, and the WARNING,INFO,DEBUG,VERBOSE levels can be enabled here.

    config ESP8266_WIFI_DEBUG_LOG_ERROR
        bool "Error"
    config ESP8266_WIFI_DEBUG_LOG_WARNING
        bool "Warning"
    config ESP8266_WIFI_DEBUG_LOG_INFO
        bool "Info"
    config ESP8266_WIFI_DEBUG_LOG_DEBUG
        bool "Debug"
    config ESP8266_WIFI_DEBUG_LOG_VERBOSE
        bool "Verbose"
endchoice
*/
//#define CONFIG_ESP8266_WIFI_DEBUG_LOG_ERROR 1

/*
config ESP8266_WIFI_DEBUG_LOG_SUBMODULE
    depends on ESP8266_WIFI_DEBUG_LOG_ENABLE
    bool "WiFi debug log submodule"
    default n
    help
        Enable this option to set the WiFi debug log submodule.
        Currently the log submodule contains the following parts: INIT,IOCTL,CONN,SCAN.
        The INIT submodule indicates the initialization process.The IOCTL submodule indicates the API calling
        process.
        The CONN submodule indicates the connecting process.The SCAN submodule indicates the scaning process.
*/
#define CONFIG_ESP8266_WIFI_DEBUG_LOG_SUBMODULE 0

/*
config ESP8266_WIFI_DEBUG_LOG_SUBMODULE_CORE
    depends on ESP8266_WIFI_DEBUG_LOG_SUBMODULE
    bool "core"
    default n
    help
        When this option is enabled, log for core module will be enabled..
*/
#define CONFIG_ESP8266_WIFI_DEBUG_LOG_SUBMODULE_CORE 0

/*
config ESP8266_WIFI_DEBUG_LOG_SUBMODULE_SCAN
    depends on ESP8266_WIFI_DEBUG_LOG_SUBMODULE
    bool "scan"
    default n
    help
        When this option is enabled, log for scan module will be enabled.
*/
#define CONFIG_ESP8266_WIFI_DEBUG_LOG_SUBMODULE_SCAN 0

/*
config ESP8266_WIFI_DEBUG_LOG_SUBMODULE_PM
    depends on ESP8266_WIFI_DEBUG_LOG_SUBMODULE
    bool "power management"
    default n
    help
        When this option is enabled, log for power management module will be enabled.
*/
#define CONFIG_ESP8266_WIFI_DEBUG_LOG_SUBMODULE_PM 0

/*
config ESP8266_WIFI_DEBUG_LOG_SUBMODULE_NVS
    depends on ESP8266_WIFI_DEBUG_LOG_SUBMODULE
    bool "NVS"
    default n
    help
        When this option is enabled, log for NVS module will be enabled.
*/
#define CONFIG_ESP8266_WIFI_DEBUG_LOG_SUBMODULE_NVS 0

/*
config ESP8266_WIFI_DEBUG_LOG_SUBMODULE_TRC
    depends on ESP8266_WIFI_DEBUG_LOG_SUBMODULE
    bool "TRC"
    default n
    help
        When this option is enabled, log for TRC module will be enabled.
*/
#define CONFIG_ESP8266_WIFI_DEBUG_LOG_SUBMODULE_TRC 0

/*
config ESP8266_WIFI_DEBUG_LOG_SUBMODULE_EBUF
    depends on ESP8266_WIFI_DEBUG_LOG_SUBMODULE
    bool "EBUF"
    default n
    help
        When this option is enabled, log for EBUF module will be enabled.
*/
#define CONFIG_ESP8266_WIFI_DEBUG_LOG_SUBMODULE_EBUF 0

/*
config ESP8266_WIFI_DEBUG_LOG_SUBMODULE_NET80211
    depends on ESP8266_WIFI_DEBUG_LOG_SUBMODULE
    bool "NET80211"
    default n
    help
        When this option is enabled, log for net80211 module will be enabled.
*/
#define CONFIG_ESP8266_WIFI_DEBUG_LOG_SUBMODULE_NET80211 0

/*
config ESP8266_WIFI_DEBUG_LOG_SUBMODULE_TIMER
    depends on ESP8266_WIFI_DEBUG_LOG_SUBMODULE
    bool "TIMER"
    default n
    help
        When this option is enabled, log for timer module will be enabled.
*/
#define CONFIG_ESP8266_WIFI_DEBUG_LOG_SUBMODULE_TIMER 0

/*
config ESP8266_WIFI_DEBUG_LOG_SUBMODULE_ESPNOW
    depends on ESP8266_WIFI_DEBUG_LOG_SUBMODULE
    bool "ESPNOW"
    default n
    help
        When this option is enabled, log for espnow module will be enabled.
*/
#define CONFIG_ESP8266_WIFI_DEBUG_LOG_SUBMODULE_ESPNOW 0

/*
config ESP8266_WIFI_DEBUG_LOG_SUBMODULE_MAC
    depends on ESP8266_WIFI_DEBUG_LOG_SUBMODULE
    bool "MAC layer"
    default n
    help
        When this option is enabled, log for mac layer module will be enabled.
*/
#define CONFIG_ESP8266_WIFI_DEBUG_LOG_SUBMODULE_MAC 0

/*
config ESP8266_WIFI_DEBUG_LOG_SUBMODULE_WPA
    depends on ESP8266_WIFI_DEBUG_LOG_SUBMODULE
    bool "wpa"
    default n
    help
        When this option is enabled, log for wpa module will be enabled.
*/
#define CONFIG_ESP8266_WIFI_DEBUG_LOG_SUBMODULE_WPA 0

/*
config ESP8266_WIFI_DEBUG_LOG_SUBMODULE_WPS
    depends on ESP8266_WIFI_DEBUG_LOG_SUBMODULE
    bool "wps"
    default n
    help
        When this option is enabled, log for wps module will be enabled.
*/
#define CONFIG_ESP8266_WIFI_DEBUG_LOG_SUBMODULE_WPS 0

/*
config ESP_PHY_CALIBRATION_AND_DATA_STORAGE
    bool "Store phy calibration data in NVS"
    default y
    help
        If this option is enabled, NVS will be initialized and calibration data will be loaded from there.
        PHY calibration will be skipped on deep sleep wakeup. If calibration data is not found, full calibration
        will be performed and stored in NVS. Normally, only partial calibration will be performed. 
        If this option is disabled, full calibration will be performed.

        If it's easy that your board calibrate bad data, choose 'n'.
        Two cases for example, you should choose 'n':
        1.If your board is easy to be booted up with antenna disconnected.
        2.Because of your board design, each time when you do calibration, the result are too unstable.
        If unsure, choose 'y'.
*/
#define CONFIG_ESP_PHY_CALIBRATION_AND_DATA_STORAGE

/*
config ESP_PHY_INIT_DATA_IN_PARTITION
    bool "Use a partition to store PHY init data"
    default n
    help
        If enabled, PHY init data will be loaded from a partition.
        When using a custom partition table, make sure that PHY data
        partition is included (type: 'data', subtype: 'phy').
        With default partition tables, this is done automatically.
        If PHY init data is stored in a partition, it has to be flashed there,
        otherwise runtime error will occur.

        If this option is not enabled, PHY init data will be embedded
        into the application binary.

        If unsure, choose 'n'.
*/
#define CONFIG_ESP_PHY_INIT_DATA_IN_PARTITION 0

// -----------------------------------------------------------------------------

// ./components/freertos/Kconfig

/*
config DISABLE_FREERTOS
    bool "Disable FreeRTOS"
    default n
    help
        Enable this option, FreeRTOS will not be compiled and linked, and the user
        can user other OS platform.
*/
//#define CONFIG_DISABLE_FREERTOS

/*
config FREERTOS_ENABLE_REENT
    bool "Enable \"reent\" function"
    default n
    select NEWLIB_ENABLE
    help
        Enable "reent" function and FreeRTOS should use "reent" function of newlib.

        The configuration will enable newlib.
*/
//#define CONFIG_FREERTOS_ENABLE_REENT

/*
config FREERTOS_HZ
    int "Tick rate (Hz)"
    range 1 1000
    default 100
    help
        Select the tick rate at which FreeRTOS does pre-emptive context switching.

        The value must be the divisor of 1000, otherwise it may cause time mistake.
*/
#define CONFIG_FREERTOS_HZ 1000

/*
config FREERTOS_MAX_HOOK
    int "FreeRTOS hook max number"
    range 1 16
    default 2
    help
        configurate the max number of FreeRTOS hook function.
*/
#define CONFIG_FREERTOS_MAX_HOOK 2

/*
config FREERTOS_ISR_STACKSIZE
    int "ISR stack size"
    range 512 8192
    default 512
    help
        The interrupt handlers have their own stack. The size of the stack can be defined here.
*/
#define CONFIG_FREERTOS_ISR_STACKSIZE 512

/*
config FREERTOS_EXTENED_HOOKS
    bool "Use FreeRTOS extened hooks"
    default n
    help
        By using the "esp_register_freertos_xxx_hook system", extened hook function offers a number
        of registerable hooks/callback functions that are called when a timer tick happens,
        the idle thread runs etc.

        Otherwise User can also use FreeRTOS raw hook functions "vApplicationIdleHook" and
        "vApplicationTickHook".
*/
//#define CONFIG_FREERTOS_EXTENED_HOOKS

/*
config FREERTOS_TIMER_STACKSIZE
    int "Timer stack size"
    default 2048
    help
        The size of the stack used by the timer in FreeRTOS.
*/
#define CONFIG_FREERTOS_TIMER_STACKSIZE 2048

/*
config TASK_SWITCH_FASTER
    bool "Task switch faster"
    default y
    help
        Enable this option, linking task switch function and its father functions to IRAM
        to speed up task switch. It is specific for faster I/O application and so on.

        But it may cost more 1KB IRAM, so user global heap may decrease 1KB.
*/
#define CONFIG_TASK_SWITCH_FASTER

/*
config USE_QUEUE_SETS
    bool "Using Queue Sets"
    default n
    help
        Enable this option, the FreeRTOS macro "configUSE_QUEUE_SETS" in file "FreeRTOSConfig.h" will be set to be 1.
*/
#define CONFIG_USE_QUEUE_SETS 0

/*
config ENABLE_FREERTOS_SLEEP
    bool "Enable FreeRTOS SLEEP"
    default n
    help
        Enable this option, FreeRTOS sleep module at idle task will be enable.
        
        The module is not working now, so if users want to make it work, they should do this themselves.
*/
#define CONFIG_ENABLE_FREERTOS_SLEEP 0

/*
config FREERTOS_USE_TRACE_FACILITY
    bool "Enable FreeRTOS trace facility"
    default n
    help
        If enabled, configUSE_TRACE_FACILITY will be defined as 1 in FreeRTOS.
        This will allow the usage of trace facility functions such as
        uxTaskGetSystemState().
*/
//#define CONFIG_FREERTOS_USE_TRACE_FACILITY

/*
config FREERTOS_USE_STATS_FORMATTING_FUNCTIONS
    bool "Enable FreeRTOS stats formatting functions"
    depends on FREERTOS_USE_TRACE_FACILITY
    default n
    help
        If enabled, configUSE_STATS_FORMATTING_FUNCTIONS will be defined as 1 in
        FreeRTOS. This will allow the usage of stats formatting functions such
        as vTaskList().
*/
//#define CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS

/*
config FREERTOS_GENERATE_RUN_TIME_STATS
    bool "Enable FreeRTOS to collect run time stats"
    default n
    select FREERTOS_USE_TRACE_FACILITY
    select FREERTOS_USE_STATS_FORMATTING_FUNCTIONS
    help
        If enabled, configGENERATE_RUN_TIME_STATS will be defined as 1 in
        FreeRTOS. This will allow FreeRTOS to collect information regarding the
        usage of processor time amongst FreeRTOS tasks. Run time stats are
        generated using either the ESP Timer or the CPU Clock as the clock
        source (Note that run time stats are only valid until the clock source
        overflows). The function vTaskGetRunTimeStats() will also be available
        if FREERTOS_USE_STATS_FORMATTING_FUNCTIONS and
        FREERTOS_USE_TRACE_FACILITY are enabled. vTaskGetRunTimeStats() will
        display the run time of each task as a % of the total run time of all
        CPUs (task run time / no of CPUs) / (total run time / 100 )
*/
//#define CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS

/*
choice FREERTOS_RUN_TIME_STATS_CLK
    prompt "Choose the clock source for run time stats"
    depends on FREERTOS_GENERATE_RUN_TIME_STATS
    default FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER
    help
        Choose the clock source for FreeRTOS run time stats. Options are CPU0's
        CPU Clock or the ESP Timer. Both clock sources are 32 bits. The CPU
        Clock can run at a higher frequency hence provide a finer resolution
        but will overflow much quicker. Note that run time stats are only valid
        until the clock source overflows.

config FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER
    bool "Use ESP TIMER for run time stats"
    help
        ESP Timer will be used as the clock source for FreeRTOS run time stats.
        The ESP Timer runs at a frequency of 1MHz regardless of Dynamic
        Frequency Scaling. Therefore the ESP Timer will overflow in
        approximately 4290 seconds.

config FREERTOS_RUN_TIME_STATS_USING_CPU_CLK
    bool "Use CPU Clock for run time stats"
    help
        CPU Clock will be used as the clock source for the generation of run
        time stats. The CPU Clock has a frequency dependent on
        ESP32_DEFAULT_CPU_FREQ_MHZ and Dynamic Frequency Scaling (DFS).
        Therefore the CPU Clock frequency can fluctuate between 80 to 240MHz.
        Run time stats generated using the CPU Clock represents the number of
        CPU cycles each task is allocated and DOES NOT reflect the amount of
        time each task runs for (as CPU clock frequency can change). If the CPU
        clock consistently runs at the maximum frequency of 240MHz, it will
        overflow in approximately 17 seconds.

endchoice
*/
//#define CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER

// -----------------------------------------------------------------------------

// ./components/log/Kconfig

/*
choice LOG_DEFAULT_LEVEL
   bool "Default log verbosity"
   default LOG_DEFAULT_LEVEL_INFO
   help
       Specify how much output to see in logs by default.
       You can set lower verbosity level at runtime using
       esp_log_level_set function.
       
       Note that this setting limits which log statements
       are compiled into the program. So setting this to, say,
       "Warning" would mean that changing log level to "Debug"
       at runtime will not be possible.

config LOG_DEFAULT_LEVEL_NONE
   bool "No output"
config LOG_DEFAULT_LEVEL_ERROR
   bool "Error"
config LOG_DEFAULT_LEVEL_WARN
   bool "Warning"
config LOG_DEFAULT_LEVEL_INFO
   bool "Info"
config LOG_DEFAULT_LEVEL_DEBUG
   bool "Debug"
config LOG_DEFAULT_LEVEL_VERBOSE
   bool "Verbose"
endchoice

config LOG_DEFAULT_LEVEL
    int
    default 0 if LOG_DEFAULT_LEVEL_NONE
    default 1 if LOG_DEFAULT_LEVEL_ERROR
    default 2 if LOG_DEFAULT_LEVEL_WARN
    default 3 if LOG_DEFAULT_LEVEL_INFO
    default 4 if LOG_DEFAULT_LEVEL_DEBUG
    default 5 if LOG_DEFAULT_LEVEL_VERBOSE
*/
//#define CONFIG_LOG_DEFAULT_LEVEL 0
#define CONFIG_LOG_DEFAULT_LEVEL 5

/*
config LOG_COLORS
   bool "Use ANSI terminal colors in log output"
   default "y"
   help
      Enable ANSI terminal color codes in bootloader output.

      In order to view these, your terminal program must support ANSI color codes.
*/
#define CONFIG_LOG_COLORS

/*
config LOG_SET_LEVEL
    bool "Enable log set level"
    default n
    help
        Enable this option, user can set tag level.
*/
//#define CONFIG_LOG_SET_LEVEL

// -----------------------------------------------------------------------------

// ./components/lwip/Kconfig

/*
config LWIP_USE_IRAM
    bool "Enable lwip use iram option"
    default n
*/
#define CONFIG_LWIP_USE_IRAM 0

/*
config TCPIP_RECVMBOX_SIZE
    int "TCPIP task receive mail box size"
    default 32
    range 6 64
    help
        Set TCPIP task receive mail box size. Generally bigger value means higher throughput
        but more memory. The value should be bigger than UDP/TCP mail box size.
*/
#define CONFIG_TCPIP_RECVMBOX_SIZE 32

/*
config LWIP_ARP_TABLE_SIZE
    int "Number of active MAC-IP address pairs cached"
    range 1 16
    default 10
*/
#define CONFIG_LWIP_ARP_TABLE_SIZE 10

/*
config LWIP_ARP_MAXAGE
    int "The time an ARP entry stays valid after its last update"
    range 100 65535
    default 300
*/
#define CONFIG_LWIP_ARP_MAXAGE 300

/*
config LWIP_IPV6_MLD_SOCK
    bool "LWIP socket supports IPv6 multicast configuration"
    default y
    depends on LWIP_IPV6
    help
        Enable the option can enable LWIP socket IPv6 multicast configuration.
*/
//#define CONFIG_LWIP_IPV6_MLD_SOCK

/*
config LWIP_SOCKET_MULTITHREAD
    bool "LWIP socket supports multithread"
    default y
    help
        Enable the option can enable LWIP socket multithread and all
        function will be thread safe.
*/
#define CONFIG_LWIP_SOCKET_MULTITHREAD

/*
config ENABLE_NONBLOCK_SPEEDUP
    bool "Speed up send speed of nonblock TCP"
    default n
    help
        Enable this option, send speed of nonblock TCP will increase obviously.
*/
#define CONFIG_ENABLE_NONBLOCK_SPEEDUP 0

/*
config SET_SOLINGER_DEFAULT
    bool "set socket SO_LINGER default"
    default y
    depends on LWIP_SOCKET_MULTITHREAD
    help
        The function is only used by socket multi-thread.

        Enable this option can set the target socket to enable the "SO_LINGER" and config timeout to be "0" when it is created.
        It means that if close the socket, all send queue will be dropped, so heap memory can be collected immediately,
        but some packets which are waiting to be sent will lost.
*/
#define CONFIG_SET_SOLINGER_DEFAULT 1

/*
config ESP_UDP_SYNC_SEND
    bool "LWIP socket UDP sync send"
    default y
    help
        Enable the option can enable LWIP socket UDP sync send. CPU cost
        should decrease but memory cost increase and it can make UDP
        throughput increase a lot.
*/
#define CONFIG_ESP_UDP_SYNC_SEND

/*
config ESP_UDP_SYNC_RETRY_MAX
    int "LWIP socket UDP sync send retry max count"
    range 1 10
    default 5
    depends on ESP_UDP_SYNC_SEND
    help
        When UDP sync send count reaches the value, then the packet should
        be lost and LWIP core thread wake up the up-level send thread.
*/
#define CONFIG_ESP_UDP_SYNC_RETRY_MAX 5

/*
config LWIP_MAX_SOCKETS
    int "Max number of open sockets"
    range 1 16
    default 10
    help
        Sockets take up a certain amount of memory, and allowing fewer
        sockets to be open at the same time conserves memory. Specify
        the maximum amount of sockets here. The valid value is from 1
        to 16.
*/
#define CONFIG_LWIP_MAX_SOCKETS 10

/*
config LWIP_SO_REUSE
    bool "Enable SO_REUSEADDR option"
    default y
    help
        Enabling this option allows binding to a port which remains in
        TIME_WAIT.
*/
#define CONFIG_LWIP_SO_REUSE 1

/*
config LWIP_SO_REUSE_RXTOALL
    bool "SO_REUSEADDR copies broadcast/multicast to all matches"
    depends on LWIP_SO_REUSE
    default y
    help
        Enabling this option means that any incoming broadcast or multicast
        packet will be copied to all of the local sockets that it matches
        (may be more than one if SO_REUSEADDR is set on the socket.)

        This increases memory overhead as the packets need to be copied,
        however they are only copied per matching socket. You can safely
        disable it if you don't plan to receive broadcast or multicast
        traffic on more than one socket at a time.
*/
#define CONFIG_LWIP_SO_REUSE_RXTOALL 1

/*
config LWIP_SO_RCVBUF
    bool "Enable SO_RCVBUF option"
    default n
    help
        Enabling this option allows checking for available data on a netconn.
*/
#define CONFIG_LWIP_SO_RCVBUF 0

/*
config LWIP_RECV_BUFSIZE_DEFAULT
    int "The default value for recv_bufsize"
    default 11680
    range 2920 11680
*/
#define CONFIG_LWIP_RECV_BUFSIZE_DEFAULT 11680

/*
config LWIP_TCP_CLOSE_TIMEOUT_MS_DEFAULT
    int "TCP socket/netconn close waits time to send the FIN"
    default 10000
    range 10000 20000
*/
#define CONFIG_LWIP_TCP_CLOSE_TIMEOUT_MS_DEFAULT 10000

/*
config LWIP_IP_FRAG
    bool "Enable fragment outgoing IP packets"
    default n
    help
        Enabling this option allows fragmenting outgoing IP packets if their size
        exceeds MTU.
*/
#define CONFIG_LWIP_IP_FRAG 0

/*
menuconfig LWIP_IP_REASSEMBLY
    bool "Enable reassembly incoming fragmented IP packets"
    default n
    help
        Enabling this option allows reassemblying incoming fragmented IP packets.
*/
#define CONFIG_LWIP_IP_REASSEMBLY 0

/*
config LWIP_IP_REASS_MAX_PBUFS
    int "Total maximum amount of pbufs waiting to be reassembled"
    default 10
    range 1 16
*/
#define CONFIG_LWIP_IP_REASS_MAX_PBUFS 10

/*
config LWIP_IP_SOF_BROADCAST
    bool "Enable broadcast filter per pcb on udp and raw send operation"
    default n
    help
*/
#define CONFIG_LWIP_IP_SOF_BROADCAST 0

/*
config LWIP_IP_SOF_BROADCAST_RECV
    bool "Enable the broadcast filter on recv operations"
    default n
    help
*/
#define CONFIG_LWIP_IP_SOF_BROADCAST_RECV 0

/*
menuconfig LWIP_ICMP
    bool "ICMP"
    default y
    help
*/
#define CONFIG_LWIP_ICMP 1

/*
config LWIP_MULTICAST_PING
   bool "Respond to multicast pings"
   default n
   depends on LWIP_ICMP
*/
#define CONFIG_LWIP_MULTICAST_PING 0

/*
config LWIP_BROADCAST_PING
   bool "Respond to broadcast pings"
   default n
   depends on LWIP_ICMP
*/
#define CONFIG_LWIP_BROADCAST_PING 0

/*
config LWIP_RAW
   bool "Enable application layer to hook into the IP layer itself"
   default n
*/
#define CONFIG_LWIP_RAW 0

/*
config LWIP_DHCP_DOES_ARP_CHECK
    bool "DHCP: Perform ARP check on any offered address"
    default y
    help
        Enabling this option performs a check (via ARP request) if the offered IP address
        is not already in use by another host on the network.
*/
#define CONFIG_LWIP_DHCP_DOES_ARP_CHECK 1

/*
config LWIP_DHCP_MAX_NTP_SERVERS
    int "Maximum number of NTP servers"
    default 1
    range 1 8
    help
        Set maximum number of NTP servers used by LwIP SNTP module.
        First argument of sntp_setserver/sntp_setservername functions
        is limited to this value.
*/
#define CONFIG_LWIP_DHCP_MAX_NTP_SERVERS 1

/*
config LWIP_DHCPS_LEASE_UNIT
    int "Multiplier for lease time, in seconds"
    range 1 3600
    default 60
    help
        The DHCP server is calculating lease time multiplying the sent 
        and received times by this number of seconds per unit. 
        The default is 60, that equals one minute.
*/
#define CONFIG_LWIP_DHCPS_LEASE_UNIT 60

/*
config LWIP_DHCPS_MAX_STATION_NUM
    int "Maximum number of stations"
    range 1 8
    default 8
    help
        The maximum number of DHCP clients that are connected to the server.
        After this number is exceeded, DHCP server removes of the oldest device
        from it's address pool, without notification.
*/
#define CONFIG_LWIP_DHCPS_MAX_STATION_NUM 8

/*
menuconfig LWIP_AUTOIP
    bool "Enable IPV4 Link-Local Addressing (AUTOIP)"
    default n
    help
        Enabling this option allows the device to self-assign an address
        in the 169.256/16 range if none is assigned statically or via DHCP.

        See RFC 3927.
*/
#define CONFIG_LWIP_AUTOIP 0

/*
config LWIP_DHCP_AUTOIP_COOP_TRIES
    int "DHCP Probes before self-assigning IPv4 LL address"
    range 1 100
    default 2
    depends on LWIP_AUTOIP
    help
        DHCP client will send this many probes before self-assigning a
        link local address.

        From LWIP help: "This can be set as low as 1 to get an AutoIP
        address very quickly, but you should be prepared to handle a
        changing IP address when DHCP overrides AutoIP."
*/
//#define CONFIG_LWIP_DHCP_AUTOIP_COOP_TRIES 2

/*
config LWIP_AUTOIP_MAX_CONFLICTS
    int "Max IP conflicts before rate limiting"
    range 1 100
    default 9
    depends on LWIP_AUTOIP
    help
        If the AUTOIP functionality detects this many IP conflicts while
        self-assigning an address, it will go into a rate limited mode.
*/
//#define CONFIG_LWIP_AUTOIP_MAX_CONFLICTS 9

/*
config LWIP_AUTOIP_RATE_LIMIT_INTERVAL
    int "Rate limited interval (seconds)"
    range 5 120
    default 20
    depends on LWIP_AUTOIP
    help
        If rate limiting self-assignment requests, wait this long between
        each request.
*/
//#define CONFIG_LWIP_AUTOIP_RATE_LIMIT_INTERVAL 20

/*
config LWIP_IGMP
    bool "Enable IGMP module"
    default y
*/
#define CONFIG_LWIP_IGMP 1

/*
config DNS_MAX_SERVERS
    int "The maximum of DNS servers"
    range 1 5
    default 2
*/
#define CONFIG_DNS_MAX_SERVERS 2

/*
menuconfig LWIP_NETIF_LOOPBACK
    bool "Enable per-interface loopback"
    default n
    help
        Enabling this option means that if a packet is sent with a destination
        address equal to the interface's own IP address, it will "loop back" and
        be received by this interface.
*/
#define CONFIG_LWIP_NETIF_LOOPBACK 0

/*
config LWIP_LOOPBACK_MAX_PBUFS
    int "Max queued loopback packets per interface"
    range 0 16
    default 0
    depends on LWIP_NETIF_LOOPBACK
    help
         Configure the maximum number of packets which can be queued for
         loopback on a given interface. Reducing this number may cause packets
         to be dropped, but will avoid filling memory with queued packet data.
*/
//#define CONFIG_LWIP_LOOPBACK_MAX_PBUFS 0

/*
config TCP_HIGH_SPEED_RETRANSMISSION
    bool "TCP high speed retransmissions"
    default n
    help
        "Enable this option, TCP retransmissions time will always be set to 500ms forcely."
*/
#define CONFIG_TCP_HIGH_SPEED_RETRANSMISSION 0

/*
config LWIP_MAX_ACTIVE_TCP
    int "Maximum active TCP Connections"
    range 1 32
    default 5
    help
        The maximum number of simultaneously active TCP
        connections. The practical maximum limit is
        determined by available heap memory at runtime.

        Changing this value by itself does not substantially
        change the memory usage of LWIP, except for preventing
        new TCP connections after the limit is reached.
*/
#define CONFIG_LWIP_MAX_ACTIVE_TCP 5

/*
config LWIP_MAX_LISTENING_TCP
    int "Maximum listening TCP Connections"
    range 1 16
    default 8
    help
        The maximum number of simultaneously listening TCP
        connections. The practical maximum limit is
        determined by available heap memory at runtime.

        Changing this value by itself does not substantially
        change the memory usage of LWIP, except for preventing
        new listening TCP connections after the limit is reached.
*/
#define CONFIG_LWIP_MAX_LISTENING_TCP 8

/*
config TCP_MAXRTX
    int "Maximum number of retransmissions of data segments"
    default 12
    range 3 12
    help
        Set maximum number of retransmissions of data segments.
*/
#define CONFIG_TCP_MAXRTX 12

/*
config TCP_SYNMAXRTX
    int "Maximum number of retransmissions of SYN segments"
    default 6
    range 3 12
    help
        Set maximum number of retransmissions of SYN segments.
*/
#define CONFIG_TCP_SYNMAXRTX 6

/*
config TCP_MSS
    int "Maximum Segment Size (MSS)"
    default 1460
    range 536 1460
    help
        Set maximum segment size for TCP transmission.

        Can be set lower to save RAM, the default value 1436 will give best throughput.
*/
#define CONFIG_TCP_MSS 1460

/*
config TCP_SND_BUF_DEFAULT
    int "Default send buffer size"
    default 2920  # 2 * default MSS
    range 2920 11680
    help
        Set default send buffer size for new TCP sockets.

        Per-socket send buffer size can be changed at runtime
        with lwip_setsockopt(s, TCP_SNDBUF, ...).

        This value must be at least 2x the MSS size, and the default
        is 4x the default MSS size.

        Setting a smaller default SNDBUF size can save some RAM, but
        will decrease performance.
*/
#define CONFIG_TCP_SND_BUF_DEFAULT 2920

/*
config TCP_WND_DEFAULT
    int "Default receive window size"
    default 5840 # 4 * default MSS
    range 2920 14600
    help
        Set default TCP receive window size for new TCP sockets.

        Per-socket receive window size can be changed at runtime
        with lwip_setsockopt(s, TCP_WINDOW, ...).

        Setting a smaller default receive window size can save some RAM,
        but will significantly decrease performance.
*/
#define CONFIG_TCP_WND_DEFAULT 5840

/*
config TCP_RECVMBOX_SIZE
    int "Default TCP receive mail box size"
    default 6
    range 6 32
    help
        Set TCP receive mail box size. Generally bigger value means higher throughput
        but more memory. The recommended value is: TCP_WND_DEFAULT/TCP_MSS + 2, e.g. if 
        TCP_WND_DEFAULT=14360, TCP_MSS=1436, then the recommended receive mail box size is 
        (14360/1436 + 2) = 12.

        TCP receive mail box is a per socket mail box, when the application receives packets
        from TCP socket, LWIP core firstly posts the packets to TCP receive mail box and the 
        application then fetches the packets from mail box. It means LWIP can caches maximum 
        TCP_RECCVMBOX_SIZE packets for each TCP socket, so the maximum possible cached TCP packets
        for all TCP sockets is TCP_RECCVMBOX_SIZE multiples the maximum TCP socket number. In other
        words, the bigger TCP_RECVMBOX_SIZE means more memory.
        On the other hand, if the receiv mail box is too small, the mail box may be full. If the 
        mail box is full, the LWIP drops the packets. So generally we need to make sure the TCP
        receive mail box is big enough to avoid packet drop between LWIP core and application.
*/
#define CONFIG_TCP_RECVMBOX_SIZE 6

/*
config TCP_QUEUE_OOSEQ
    bool "Queue incoming out-of-order segments"
    default n
    help
        Queue incoming out-of-order segments for later use.

        Disable this option to save some RAM during TCP sessions, at the expense
        of increased retransmissions if segments arrive out of order.
*/
#define CONFIG_TCP_QUEUE_OOSEQ 0

/*
choice TCP_OVERSIZE
    prompt "Pre-allocate transmit PBUF size"
    default TCP_OVERSIZE_MSS
    help
        Allows enabling "oversize" allocation of TCP transmission pbufs ahead of time,
        which can reduce the length of pbuf chains used for transmission.

        This will not make a difference to sockets where Nagle's algorithm
        is disabled.

        Default value of MSS is fine for most applications, 25% MSS may save
        some RAM when only transmitting small amounts of data. Disabled will
        have worst performance and fragmentation characteristics, but uses
        least RAM overall.

config TCP_OVERSIZE_MSS
     bool "MSS"
config TCP_OVERSIZE_QUARTER_MSS
     bool "25% MSS"
config TCP_OVERSIZE_DISABLE
     bool "Disabled"

endchoice
*/
#define CONFIG_TCP_OVERSIZE_MSS

/*
config LWIP_TCP_TIMESTAMPS
    bool "Support the TCP timestamp option"
    default n
    help
        The timestamp option is currently only used to help remote hosts, it is not
        really used locally. Therefore, it is only enabled when a TS option is
        received in the initial SYN packet from a remote host.
*/
#define CONFIG_LWIP_TCP_TIMESTAMPS 0

/*
config LWIP_MAX_UDP_PCBS
    int "Maximum active UDP control blocks"
    range 1 32
    default 4
    help
        The maximum number of active UDP "connections" (ie
        UDP sockets sending/receiving data).
        The practical maximum limit is determined by available
        heap memory at runtime.
*/
#define CONFIG_LWIP_MAX_UDP_PCBS 4

/*
config UDP_RECVMBOX_SIZE
    int "Default UDP receive mail box size"
    default 6
    range 6 64
    help
        Set UDP receive mail box size. The recommended value is 6.

        UDP receive mail box is a per socket mail box, when the application receives packets 
        from UDP socket, LWIP core firstly posts the packets to UDP receive mail box and the
        application then fetches the packets from mail box. It means LWIP can caches maximum
        UDP_RECCVMBOX_SIZE packets for each UDP socket, so the maximum possible cached UDP packets 
        for all UDP sockets is UDP_RECCVMBOX_SIZE multiples the maximum UDP socket number. In other
        words, the bigger UDP_RECVMBOX_SIZE means more memory.
        On the other hand, if the receiv mail box is too small, the mail box may be full. If the
        mail box is full, the LWIP drops the packets. So generally we need to make sure the UDP
        receive mail box is big enough to avoid packet drop between LWIP core and application.
*/
#define CONFIG_UDP_RECVMBOX_SIZE 6

/*
config TCPIP_TASK_STACK_SIZE
    int "TCP/IP Task Stack Size"
    default 2048
    range 2048 8192
    help
       Configure TCP/IP task stack size, used by LWIP to process multi-threaded TCP/IP operations.
       Setting this stack too small will result in stack overflow crashes.
*/
#define CONFIG_TCPIP_TASK_STACK_SIZE 2048

/*
config LWIP_MAX_RAW_PCBS
    int "Maximum LWIP RAW PCBs"
    range 1 32
    default 4
    help
        The maximum number of simultaneously active LWIP
        RAW protocol control blocks. The practical maximum
        limit is determined by available heap memory at runtime.
*/
#define CONFIG_LWIP_MAX_RAW_PCBS 4

/*
menuconfig LWIP_IPV6
    bool "Enable IPv6"
    default n
*/
#define CONFIG_LWIP_IPV6 0

/*
config LWIP_IPV6_NUM_ADDRESSES
    int "Number of IPv6 addresses per netif"
    depends on LWIP_IPV6
    range 3 5
    default 3
*/
//#define CONFIG_LWIP_IPV6_NUM_ADDRESSES 3

/*
config LWIP_IPV6_FORWARD
    bool "Forward IPv6 packets across netifs"
    depends on LWIP_IPV6
    default n
*/
//#define CONFIG_LWIP_IPV6_FORWARD 0

/*
config LWIP_IPV6_FRAG
    bool "Fragment outgoing IPv6 packets that are too big"
    depends on LWIP_IPV6
    default n
*/
//#define CONFIG_LWIP_IPV6_FRAG 0

/*
config LWIP_ND6_RDNSS_MAX_DNS_SERVERS
    bool "The IPv6 ND6 RDNSS max DNS servers"
    depends on LWIP_IPV6
    default n
    help
        Use IPv6 Router Advertisement Recursive DNS Server Option (as per RFC 6106) 
        to copy a defined maximum number of DNS servers to the DNS module
*/
//#define CONFIG_LWIP_ND6_RDNSS_MAX_DNS_SERVERS 0

/*
config LWIP_STATS
    bool "Enable statistics collection in lwip_stats"
    default n
*/
#define CONFIG_LWIP_STATS 0

/*
config ESP_LWIP_MEM_DBG
    bool "Enable LWIP memory debug"
    default n
    help
        Enable this option, LWIP allocated memory information can be traced by "heap" components.

        User can call "heap_trace_start(HEAP_TRACE_LEAKS)" to start tracing and call "heap_trace_dump()"
        to list the memory map.
*/
//#define CONFIG_ESP_LWIP_MEM_DBG

/*
menuconfig LWIP_DEBUG
    bool "Enable lwip Debug"
    default n
*/
#define CONFIG_LWIP_DEBUG 0

/*
config LWIP_ETHARP_DEBUG
    bool "Enable debugging in etharp.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_ETHARP_DEBUG 0

/*
config LWIP_NETIF_DEBUG
    bool "Enable debugging in netif.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_NETIF_DEBUG 0

/*
config LWIP_PBUF_DEBUG
    bool "Enable debugging in pbuf.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_PBUF_DEBUG 0

/*
config LWIP_API_LIB_DEBUG
    bool "Enable debugging in api_lib.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_API_LIB_DEBUG 0

/*
config LWIP_API_MSG_DEBUG
    bool "Enable debugging in api_msg.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_API_MSG_DEBUG 0

/*
config LWIP_SOCKETS_DEBUG
    bool "Enable debugging in sockets.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_SOCKETS_DEBUG 0

/*
config LWIP_ICMP_DEBUG
    bool "Enable debugging in icmp.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_ICMP_DEBUG 0

/*
config LWIP_IGMP_DEBUG
    bool "Enable debugging in igmp.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_IGMP_DEBUG 0

/*
config LWIP_INET_DEBUG
    bool "Enable debugging in inet.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_INET_DEBUG 0

/*
config LWIP_ETHERNETIF_DEBUG
    bool "Enable debugging in ethernetif.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_ETHERNETIF_DEBUG 0

/*
config LWIP_IP_DEBUG
    bool "Enable debugging for IP"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_IP_DEBUG 0

/*
config LWIP_IP_REASS_DEBUG
    bool "Enable debugging in ip_frag.c for both frag & reass"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_IP_REASS_DEBUG 0

/*
config LWIP_RAW_DEBUG
    bool "Enable debugging in raw.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_RAW_DEBUG 0

/*
config LWIP_MEM_DEBUG
    bool "Enable debugging in mem.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_MEM_DEBUG 0

/*
config LWIP_MEMP_DEBUG
    bool "Enable debugging in memp.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_MEMP_DEBUG 0

/*
config LWIP_SYS_DEBUG
    bool "Enable debugging in sys.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_SYS_DEBUG 0

/*
config LWIP_TIMERS_DEBUG
    bool "Enable debugging in timers.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_TIMERS_DEBUG 0

/*
config LWIP_TCP_DEBUG
    bool "Enable debugging for TCP"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_TCP_DEBUG 0

/*
config LWIP_TCP_INPUT_DEBUG
    bool "Enable debugging in tcp_in.c for incoming debug"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_TCP_INPUT_DEBUG 0

/*
config LWIP_TCP_FR_DEBUG
    bool "Enable debugging in tcp_in.c for fast retransmit"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_TCP_FR_DEBUG 0

/*
config LWIP_TCP_RTO_DEBUG
    bool "Enable debugging in TCP for retransmit"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_TCP_RTO_DEBUG 0

/*
config LWIP_TCP_CWND_DEBUG
    bool "Enable debugging for TCP congestion window"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_TCP_CWND_DEBUG 0

/*
config LWIP_TCP_WND_DEBUG
    bool "Enable debugging in tcp_in.c for window updating"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_TCP_WND_DEBUG 0

/*
config LWIP_TCP_OUTPUT_DEBUG
    bool "Enable debugging in tcp_out.c output functions"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_TCP_OUTPUT_DEBUG 0

/*
config LWIP_TCP_RST_DEBUG
    bool "Enable debugging for TCP with the RST message"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_TCP_RST_DEBUG 0

/*
config LWIP_TCP_QLEN_DEBUG
    bool "Enable debugging for TCP queue lengths"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_TCP_QLEN_DEBUG 0

/*
config LWIP_UDP_DEBUG
    bool "Enable debugging in UDP"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_UDP_DEBUG 0

/*
config LWIP_TCPIP_DEBUG
    bool "Enable debugging in tcpip.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_TCPIP_DEBUG 0

/*
config LWIP_SLIP_DEBUG
    bool "Enable debugging in slipif.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_SLIP_DEBUG 0

/*
config LWIP_DHCP_DEBUG
    bool "Enable debugging in dhcp.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_DHCP_DEBUG 0

/*
config LWIP_DHCP_SERVER_DEBUG
    bool "Enable debugging in dhcpserver.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_DHCP_SERVER_DEBUG 0

/*
config LWIP_AUTOIP_DEBUG
    bool "Enable debugging in autoip.c"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_AUTOIP_DEBUG 0

/*
config LWIP_DNS_DEBUG
    bool "Enable debugging for DNS"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_DNS_DEBUG 0

/*
config LWIP_IP6_DEBUG
    bool "Enable debugging for IPv6"
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_IP6_DEBUG 0

/*
config LWIP_SNTP_DEBUG
    bool "Enable debugging for SNTP."
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_SNTP_DEBUG 0

/*
config LWIP_THREAD_SAFE_DEBUG
    bool "Enable debugging for LWIP thread safety."
    depends on LWIP_DEBUG
    default n
*/
//#define CONFIG_LWIP_THREAD_SAFE_DEBUG

/*
config LWIP_PBUF_CACHE_DEBUG
    bool "Enable debugging for LWIP pbuf cache."
    depends on LWIP_DEBUG
    default n
*/
#define CONFIG_LWIP_PBUF_CACHE_DEBUG 0

// -----------------------------------------------------------------------------

// ./components/newlib/Kconfig

/*
choice NEWLIB_LIBRARY_LEVEL
    prompt "newlib level"
    default NEWLIB_LIBRARY_LEVEL_NANO
    depends on NEWLIB_ENABLE
    help
        Choose newlib library level.

config NEWLIB_LIBRARY_LEVEL_NORMAL
    bool "normal"
    help
        If you need 64-bit integer formatting support or C99 features, select this
        option.

config NEWLIB_LIBRARY_LEVEL_NANO
    bool "nano"
    help
        The newlib library which has been compiled with so-called "nano"
        formatting option. This option doesn't support 64-bit integer formats and C99
        features, such as positional arguments.

        For more details about "nano" formatting option, please see newlib readme file,
        search for '--enable-newlib-nano-formatted-io':
        https://sourceware.org/newlib/README

        If you do not need 64-bit integer formatting support or C99 features, select this
        option.

endchoice
*/
#define CONFIG_NEWLIB_LIBRARY_LEVEL_NANO

// -----------------------------------------------------------------------------

// ./components/partition_table/Kconfig.projbuild

/*
config PARTITION_TABLE_OFFSET
    hex "Partition table offset address at flash"
    default 0x8000
    help
        The partition table cannot be placed at application address.
*/
#define CONFIG_PARTITION_TABLE_OFFSET 0x8000

// -----------------------------------------------------------------------------

// ./components/tcpip_adapter/Kconfig

/*
config IP_LOST_TIMER_INTERVAL
    int "IP Address lost timer interval (seconds)"
    range 0 65535
    default 120
    help
        The value of 0 indicates the IP lost timer is disabled, otherwise the timer is enabled.
        
        The IP address may be lost because of some reasons, e.g. when the station disconnects
        from soft-AP, or when DHCP IP renew fails etc. If the IP lost timer is enabled, it will 
        be started everytime the IP is lost. Event SYSTEM_EVENT_STA_LOST_IP will be raised if
        the timer expires. The IP lost timer is stopped if the station get the IP again before
        the timer expires.
*/
#define CONFIG_IP_LOST_TIMER_INTERVAL 120

// -----------------------------------------------------------------------------

// ./components/wpa_supplicant/Kconfig

/*
config LTM_FAST
    bool "Use faster div, esptmod, sqr, montgomery multiplication algorithm"
    default y
    help
        Enable the option can enable faster div, faster exptmod, faster sqr, fast
        montgomery multiplication algorithm. Enable this option will cost about 
        3K ROM more than disable this option.
*/
#define CONFIG_LTM_FAST 1

// -----------------------------------------------------------------------------

// ./components/esptool_py/Kconfig.projbuild

/*
choice FLASHMODE
    prompt "Flash SPI mode"
    default FLASHMODE_QIO
    help
        Mode the flash chip is flashed in, as well as the default mode for the
        binary to run in.

        The esptool.py flashes firmware at DIO mode when user select "QIO", "QOUT" or "DIO" mode.

config FLASHMODE_QIO
    bool "QIO"
config FLASHMODE_QOUT
    bool "QOUT"
config FLASHMODE_DIO
    bool "DIO"
config FLASHMODE_DOUT
    bool "DOUT"
endchoice
*/
#define CONFIG_FLASHMODE_QIO 1

/*
config SPI_FLASH_MODE
    hex
    default 0x0 if FLASHMODE_QIO
    default 0x1 if FLASHMODE_QOUT
    default 0x2 if FLASHMODE_DIO
    default 0x3 if FLASHMODE_DOUT
*/
#define CONFIG_SPI_FLASH_MODE 0x0

/*
config SPI_FLASH_FREQ
    hex
    default 0xf if ESPTOOLPY_FLASHFREQ_80M
    default 0x0 if ESPTOOLPY_FLASHFREQ_40M
    default 0x1 if ESPTOOLPY_FLASHFREQ_26M
    default 0x2 if ESPTOOLPY_FLASHFREQ_20M
*/
#define CONFIG_SPI_FLASH_FREQ 0x0

/*
config SPI_FLASH_SIZE
    hex
    default 0x100000 if ESPTOOLPY_FLASHSIZE_1MB
    default 0x200000 if ESPTOOLPY_FLASHSIZE_2MB
    default 0x400000 if ESPTOOLPY_FLASHSIZE_4MB
    default 0x800000 if ESPTOOLPY_FLASHSIZE_8MB
    default 0x1000000 if ESPTOOLPY_FLASHSIZE_16MB
*/
#define CONFIG_SPI_FLASH_SIZE 0x200000

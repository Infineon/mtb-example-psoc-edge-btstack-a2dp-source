[Click here](../README.md) to view the README.

## Design and implementation

The design of this application is kept straightforward to help the user get started with code examples on PSOC&trade; Edge MCU devices. All PSOC&trade; Edge E84 MCU applications have a dual-CPU three-project structure to develop code for the CM33 and CM55 cores. The CM33 core has two separate projects for the secure processing environment (SPE) and non-secure processing environment (NSPE). A project folder consists of various subfolders, each denoting a specific aspect of the project. The three project folders are as follows:

**Table 1. Application projects**

Project | Description
--------|------------------------
*proj_cm33_s* | Project for CM33 secure processing environment (SPE)
*proj_cm33_ns* | Project for CM33 non-secure processing environment (NSPE)
*proj_cm55* | CM55 project

<br>

In this code example, at device reset, the secure boot process starts from the ROM boot with the secure enclave (SE) as the root of trust (RoT). From the secure enclave, the boot flow is passed on to the system CPU subsystem where the secure CM33 application starts. After all necessary secure configurations, the flow is passed on to the non-secure CM33 application. 

Resource initialization for this example is performed by this CM33 non-secure project. It configures the system clocks, pins, clock to peripheral connections, and other platform resources. To conserve power, the CM33 CPU uses a multi-counter watchdog timer (MCWDT) 0 as a low-power timer (LPTIMER). This integration allows the FreeRTOS to enter a tickless idle state, enabling the device to transition into deep sleep when the CPU is idle, minimizing power consumption. It then enables the CM55 core using the `Cy_SysEnableCM55()` function. A FreeRTOS task `cm33_blinky_task` is created, which toggles the 'User LED1' every 1000 milliseconds. 

In the CM33 non-secure application, the clocks and system resources are initialized by the BSP initialization function. The retarget-io middleware is configured to use the debug UART, and the user LED is initialized. 

**Application setup:**

The application consists of the following functionality.

| Function                                                          | Core |
| :-------------------------------------------------------          | :--- |
| Default retarget_io for printing logs using kitprog UART          | CM33 |
| Main application task - handles BT connection/disconnection       | CM33 |
| Bluetooth Stack receive/transmit Tasks                            | CM33 |
| Audio streaming thread - handles play, pause, and streaming       | CM33 |

The following files contain the necessary configurations required by application

| Macros/Variables | Configuration                                               | File |
| :----  | :-------------------------------------------          | :---------   |
| BDA_ADDRESS_BYTE_0 <br> BDA_ADDRESS_BYTE_1 <br> BDA_ADDRESS_BYTE_2 <br> BDA_ADDRESS_BYTE_3 <br> BDA_ADDRESS_BYTE_4 <br> BDA_ADDRESS_BYTE_5 | Default A2DP sink device's BDA (Bluetooth&reg; Device Address) | a2dp_source.h |
| ENABLE_BT_VERBOSE_LOGS | Enable BT debug prints for the application | a2dp_source.h |
| BT_STACK_HEAP_SIZE | Total allocated heap size for Bluetooth&reg; stack | main.c |
| BT_TASK_STACK_SIZE <br> BT_TASK_PRIORITY |  Main application task - stack size and priority | main.c |
| A2DP_TASK_STACK_SIZE <br> A2DP_TASK_PRIORITY |  Audio streaming task - stack size and priority | a2dp_source.h |
| TIMER_INTERRUPT_PRIORITY | Priority of timer interrupt | main.c |
| GPIO_INTERRUPT_PRIORITY | Priority of GPIO interrupt | board.c |
| hex_array[]   |  Raw wav file stored in array | wav_1_test.h |
| hex_array_size |  Size of raw wav file stored in array | wav_1_test.h |
| BOND_INDEX_MAX | Max number of bonded devices | app_bt_bonding.h |
| WICED_DEVICE_NAME | The name of the kit running this example visible to peer Bluetooth&reg; devices | wiced_bt_cfg.c |
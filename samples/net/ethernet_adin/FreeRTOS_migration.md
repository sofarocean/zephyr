# Changes to make moving from Zephyr to FreeRTOS

## DTS/Overlay Files
- SPI/DMA Setup
- Clock Tree Setup
- Flash mem partitioning for DFU

## Sample Application
- Joining well-known multicast groups
- Setting up sockets
  - Sending/Receiving data over sockets
- Set up sending/receiving threads

## ADIN Driver
- Define network interface
- Initialize driver
- Pipe rx/tx callbacks to L2 layer
- Signalling mechanism to service threads that tx/rx data is ready
  - Uses polling on semaphores and FIFOs

## ADIN BSP
- Threads to respond to GPIO interrupts (DATA_RDY) and completed SPI transactions
  - Associated semaphore signals and polling mechanism
- Enabling/Disabling ISRs
  - Need to use NVIC

## General
- Tracing
- GPIO Toggling
- Network/LWIP Setup
- Console 
  - Currently using UART1
- Thread priority assignments 
  - Cooperative vs Preemptive Scheduling
- Logging 
  - Log Levels
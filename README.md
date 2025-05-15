# Microros Setup
**This repository is for microros installation on STM32 Boards[F411(DISCOVERY)/F446RE(NUCLEO)]**
>**Note:** If you have already completed microros installation once then skip step 1. For official documentation click on the below link
[Microros Documentation](https://micro.ros.org/docs/tutorials/core/first_application_linux/)

# Step 1:
**Now open a new terminal/terminator and follow these steps**
# Source the ROS 2 installation
	source /opt/ros/humble/setup.bash
	
# Create a workspace and download the micro-ROS tools
	mkdir microros_ws
	cd microros_ws
	
>**Warning:** Make sure you are in the microros_ws directory until it is mentioned to change the directory

	
# Clone the repository and update dependencies using rosdep
	git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
	sudo apt update && rosdep update
	rosdep install --from-paths src --ignore-src -y
	
# Install pip
	sudo apt-get install python3-pip

# Build micro-ROS tools and source them
	colcon build
	source install/local_setup.bash
	
# Create firmware step
	ros2 run micro_ros_setup create_firmware_ws.sh host

# Download micro-ROS-Agent packages
	ros2 run micro_ros_setup create_agent_ws.sh

# Build step
	ros2 run micro_ros_setup build_agent.sh
	source install/local_setup.bash
 >**Note:** Ignore the build warnings

# Step 2: 
**Open a new STM32CubeIDE workspace via typing or using browse feature**
>**Refer to the image**
[STM32 Workspace Launch](Images/STM32_WORKSPACE.png)

**Click Launch**

>**Important:** Make sure you have logged into your STM32 Account and are connected to a personal Wi-fi.

**Click on File->New->STM32 Project.**
>**Important:** If there are packages being installed then let them get installed properly/ Agree on if any license agreement.

>**Refer to the image**
[STM32 New Project](Images/New_Project.png)
# Target selector
**Go to Board Selector and use the search bar and type NUCLEO-F446RE if you are using a nucleo board and STM32F411VET6 if you are using a discovery board.**

>**Refer to the image**
[STM32 Target Selector](Images/Target.png)

**After selection click on next->Give an appropriate Project name (EX: microros)->Do not change any other option and click on next->Finish.You would see an .ioc file like:**
>**Refer to the image**
[STM32 .ioc file](Images/IOC_file.png)

>**Note:** For initialize all peripherals with their default mode->Click Yes->Device config editor pop up -> Again click Yes.

**An .ioc file is where you can configure your STM32 Board Pins.
Click on Pinout drop down-> Select Clear Pinouts (or just use shortcut crtl+p)**
 >**Note:** For the warning that the Pinout configuration will be cleared Click Yes

**This completes initial setup.**

# Step 3: Setting up the Pinout configuration
**We need three things for installing and verifying microros which are Clock configuration, Debugging and UART**
# Clock Configuration 
**This defines the main clock(HSE) for the GPIO Pins/UART Pins and also for the LSE for the Watch dog Timer**
>**Note:** Advanced info(feel free to skip this):You can change clock settings in the Clock Configurations option where there is an interface to select which clock is used as the main clock (HSE/HSI/PLL) and which mode(Crystal Ceramic Oscillator/RC Oscillator/LC Oscillator) and also frequency can be set using frequency divders and multipliers.
 
**Click on System Core(Present in project explorer on the left hand side) ->RCC-> Select both High Speed External Clock (HSE) and Low Speed External Clock (LSE) as Crystal Ceramic Resonator this sets the main clock at 84MHz(For Nucleo board) and 100MHz(For Discovery Board).**
>**Refer to the image**
[Clock Configurations](Images/RCC.png)

# SYS settings (Debugging)
**Now again under System Core-> SYS-> Select Debug as Serial Wire,this is just settings for the debugging method(We Have Serial Wire /JTAG options but we are using STLINK Debugger which uses Serial Wire for the debugging purpose).**
# UART Communication Protocol
**To establish UART Communication protocol required for Microros follow these steps:**
# DMA Settings
>**Refer to the image**
[DMA Settings](Images/DMA.png)

**Now Under Connectivity choose->USART2->Mode as Asynchronous->Under DMA(Direct Memory Access) Settings->Click on ADD->Select USART2_Rx under the drop down->Set Priority to Very High->Also make sure under DMA Request Settings change mode from normal to circular.This setting is for the receiver.**
>**Important** Make sure under DMA Request Settings change mode from normal to circular for Rx

**Similarly we are to set it up for transmitter,Under DMA(Direct Memory Access) Settings->Click on ADD->Select USART2_Tx under the drop down->Set Priority to Very High->Also make sure under DMA Request Settings set mode as normal.**
>**Important** Make sure under DMA Request Settings change mode is at normal for Tx

**These are the DMA settings.**
# NVIC Settings
**Now let us move to NVIC(Nested Vector Interrupt Control) Table settings**

**Go to NVIC Settings->Enable the USART2 golbal interrupt.**

# Step 4: Configuring FreeRTOS settings
**Click on Middleware and Software Packs which is on the left hand side-> Under this click on FREERTOS-> Select CMSIS_V2 for interface**
**Go to Tasks and Queues and click on the only available task** 
>**Refer to the image**
[FREERTOS Settings](Images/FREERTOS.png)

**You will see an Edit Task Pop up-> Change Stack Size (Words) from 128 to 3000-> Then click OK**
>**Important** Make sure the Stack Size(Words) is atleast has more than 10 kB 

# Step 5: Microros Integration with CubeIDE
**Generate the code using Device Configuration Tool Code Generation option (It looks like a golden cog wheel/gear)**
>**Refer to the image**
[Generate Code is 1 in the image](Images/Tool.jpeg)

>**Note:** For the pop up saying open associated prespective click on Yes and for the Warnings in code generations ignore them and click on Yes

**Make sure main.c is loaded properly**

>**Note:** Now for the official repository click on the following link
[Microros Integration with CubeMX/IDE](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git)

**In files Navigate to your STM32 Project (In my case the path is /home/shishir/STM32CubeIDE/workspace_microros/microros/)**

**So the path for you might be /home/<user>/STM32CubeIDE/<workspace_name>/<project_name>/ -> right click on the free space and click on open in terminal and clone the repository given below**
>**Note:** Alternatively it can be done by using the cd CLI to navigate to this path

# Clone the Repository
	git clone https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git
 
# Setp 6: Adding Pre-build Command,include directory,precompiled library.
>**Refer to the image**
[Project Properties](Images/Properties.jpeg)

# Add Pre-build Command

**In the Project Explorer window right click on your project ->navigate to 'properties'-> C/C++ Build -> Settings -> Build Steps Tab and in Pre-build steps->Command add the following and click on apply:**

>**Note:** The short cut key to open 'properties' is to press Alt+Enter

	docker pull microros/micro_ros_static_library_builder:humble && docker run --rm -v ${workspace_loc:/${ProjName}}:/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:humble

>**Refer to the image**
[Pre-Build Command](Images/Pre_build_command.png)

# Add micro-ROS include directory

**Navigate to C/C++ Build -> Settings -> Tool Settings Tab -> MCU GCC Compiler -> Include paths**

**Under Include paths(-l) add the following command and after adding click on ok**

	../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include
>**Refer to the image**
[Include Paths](Images/paths.png)

# Add the micro-ROS precompiled library

 **Navigate to C/C++ Build -> Settings -> MCU GCC Linker -> Libraries**

 **Under Library search path(-L) add**
 
 	../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
 **Under Libraries (-l) add**
 
 	microros
 **Click on Apply and for the pop up window click on Rebuild index then you can click on Apply and Close**
# Step 7: Copying extra sources to your project
**Open files and navigate to the same path where git cloning happend under Step 5**

**You would see a folder with the name micro_ros_stm32cubemx_utils-> extra_sources-> Copy the following files:**
>custom_memory_manager.c

>microros_allocators.c

>microros_time.c

>microros_transports/dma_transport.c

**Now navigate back to your project->Core->Src->Paste the above files here**
>**Note:** If there where more files inside microros_transports/ other than dma_transport.c delete the other files(it_transport.c,udp_transport.c,usb_cdc_transport.c) after pasting them.
# Step 8: Building the Project
**Before Building the Project open a new terminal and run the following command**

	sudo chmod 666 /var/run/docker.sock
**Then click on build project which is the hammer icon on the top**
>**Refer to the image**
[Build_Project is 2 in the image](Images/Tool.jpeg)

>**Note:** This build takes approximately 30 to 45 mins and make sure you are connected to private Wi-fi connection

**Make sure that there are no errors displayed on the console after the build you may have around 48 warnings but you can ignore them**
**Again build your project but this time the buid should take only few seconds :)**

# Step 9: Create a publisher to test microros
**After a successful build we can now create a publisher**

**Navigate to the project explorer tab (left hand side) and locate micro_ros_stm32cubemx_utils->sample_main.c**

**From sample_main.c we need to copy the publisher code, but I will try to mention all the code to be copied in this step**

**Copy the following lines and in your main.c file paste it at Private includes line**
	
 	/* Private includes ----------------------------------------------------------*/
	/* USER CODE BEGIN Includes */
	#include <rcl/rcl.h>
	#include <rcl/error_handling.h>
	#include <rclc/rclc.h>
	#include <rclc/executor.h>
	#include <uxr/client/transport.h>
	#include <rmw_microxrcedds_c/config.h>
	#include <rmw_microros/rmw_microros.h>
	#include <std_msgs/msg/int32.h>
	/* USER CODE END Includes */
**Copy the following lines and in your main.c file paste it from  USER CODE BEGIN 4  to  USER CODE END 4  line**

	/* USER CODE BEGIN 4 */
	bool cubemx_transport_open(struct uxrCustomTransport * transport);
	bool cubemx_transport_close(struct uxrCustomTransport * transport);
	size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
	size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

	void * microros_allocate(size_t size, void * state);
	void microros_deallocate(void * pointer, void * state);
	void * microros_reallocate(void * pointer, size_t size, void * state);
	void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
	/* USER CODE END 4 */
**Copy the following lines and in your main.c file paste it inside the void StartDefaultTask(void argument) function,basically replace the whole content of the function with the following:**

 	 /* USER CODE BEGIN 5 */

  	// micro-ROS configuration

  	rmw_uros_set_custom_transport(
    	true,
    	(void *) &huart2,	//in sample_main.c USART3 is used so they have used &huart3 but make sure that you use &huart2(If usart2 was configured)
    	cubemx_transport_open,
    	cubemx_transport_close,
    	cubemx_transport_write,
    	cubemx_transport_read);

  	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  	freeRTOS_allocator.allocate = microros_allocate;
  	freeRTOS_allocator.deallocate = microros_deallocate;
  	freeRTOS_allocator.reallocate = microros_reallocate;
  	freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      		printf("Error on default allocators (line %d)\n", __LINE__); 
  	}

  	// micro-ROS app

  	rcl_publisher_t publisher; // This allocates memory to a varible called as publisher
  	std_msgs__msg__Int32 msg; // msg is of the type std_msgs__msg__Int32
  	rclc_support_t support; // Contains initialization data
  	rcl_allocator_t allocator; // Defines how memory will be allocated 
  	rcl_node_t node; // Represents the ROS 2 node

  	allocator = rcl_get_default_allocator();

  	//create init_options
  	rclc_support_init(&support, 0, NULL, &allocator);

  	// create node
 	 rclc_node_init_default(&node, "cubemx_node", "", &support); // A custom node called as cubemx_node is created

  	// create publisher
  	rclc_publisher_init_default(
    	&publisher,
    	&node,
    	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    	"cubemx_publisher");  // Here the node 'cubemx_node' publishes data->msg of the type Int32 to the topic cubemx_publisher

  	msg.data = 0; // Initialize

  	for(;;)
  	{
    	rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    	if (ret != RCL_RET_OK)
    	{
      		printf("Error publishing (line %d)\n", __LINE__); 
    	}
    
    	msg.data++; // Keep increamenting the value
    	osDelay(10);
  	}
  	/* USER CODE END 5 */
**Save your main.c file and build it once more, if there are no errors we just have one more step left!**

# Step 10: Verifying and testing the microros publisher(cubemx_publisher) 
**1)Connect your STM32 Board via an STM32 Cable**

**2)Now click on the drop down option in run button -> Run Configurations-> In the window which just poped up double click on STM32 C/C++ Application**

>**Refer to the image**
[Debug is 3 and Run is 4 in the image](Images/Tool.jpeg)

>**Note:** Instead of Run configuration you can also use Debug Configuration(Only do this if you know what is happen here)

**3)Now at the top you can see main,debugger,startup,source,common...Click on Debugger->  Make sure the interface is SWD->Tick the checkbox in the ST-LINK S/N option-> Click on scan-> If the borad is detected a random long number must appear (Ex:0668FF3138504B3043032130)-> Click on Apply->Then click on Run**

**4)Your code will be built again then an .elf(executable and linkable format) file would be loaded on to your board**
>**Important:** Make sure your console has similar lines

>File download complete

>Time elapsed during download operation: 00:00:02.444

>Verifying ...

>Download verified successfully

>Shutting down...

>Exit.

# TTL(Used for USB to UART Translations/Conversions) connections for STM32F411VETx(Discovery Board)
>**Note:** If you are using a Nucelo Board you can skip this step but this is a compulsary step for the Discovery Boad users

**TTL to STM32 connections are as follows:**
**1) 5V of TTL to 5V of STM32 / 3.3V of TTL to 3V of STM32**
**2) GND of TTL to GND STM32**
**3) Tx of TTL to PA3 of STM32(This would be USART_Rx as the data transmitted from there shall be recieved here)**
**4) Rx of TTL to PA2 of STM32(This would be USART_Tx as the data recieved there shall be transmitted here)**

# Connectivity to laptop/PC
**If you are using STM32F446RE(Nucleo Board) the board is capable of performing UART via the STM32 Cable, so there is no need for a TTL. So your PC and board must be connected together via the cable while performing the next part.**

**Whereas if you are using STM32F411VETx(Discovery Board) the board is not capable of performing UART via the STM32 Cable, so there is a need for a TTL. So your PC and board must be connected together via TTL only (disconnect the cable) while performing the next part.**
>**Note:(Only for Discovery Board Users)** While uploading the code you need to make use of the cable(then disconnect it) but while communicating via UART use TTL only.

# Setting up microros_ws
**Open a new terminal and type the following:**
	
 	cd microros_ws
  	source install/setup.bash 
   
**Now the following steps vary according to the board that you use:**
# For STM32F446RE(Nucleo Board)
	ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
# For STM32F411VETx(Discovey Board)
	ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
**After running these click on the STM32's reset button (Black push button) you should see the following lines if successful:**
>[1747247596.587848] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3

>[1747247596.588273] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4

>[1747247599.389385] info     | Root.cpp           | create_client            | create                 | client_key: 0x041835D4, session_id: 0x81

>[1747247599.389526] info     | SessionManager.hpp | establish_session        | session established    | client_key: 0x041835D4, address: 0

>[1747247599.444861] info     | ProxyClient.cpp    | create_participant       | participant created    | client_key: 0x041835D4, participant_id: 0x000(1)

>[1747247599.460260] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x041835D4, topic_id: 0x000(2), participant_id: 0x000(1)

>[1747247599.469178] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x041835D4, publisher_id: 0x000(3), participant_id: 0x000(1)

>[1747247599.481051] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x041835D4, datawriter_id: 0x000(5), publisher_id: 0x000(3)

>[1747247599.496145] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x041835D4, topic_id: 0x001(2), participant_id: 0x000(1)

>[1747247599.505438] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x041835D4, publisher_id: 0x001(3), participant_id: 0x000(1)

>[1747247599.516533] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x041835D4, datawriter_id: 0x001(5), publisher_id: 0x001(3)

>[1747247599.531101] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x041835D4, topic_id: 0x002(2), participant_id: 0x000(1)

>[1747247599.540301] info     | ProxyClient.cpp    | create_subscriber        | subscriber created     | client_key: 0x041835D4, subscriber_id: 0x000(4), participant_id: 0x000(1)

>[1747247599.551884] info     | ProxyClient.cpp    | create_datareader        | datareader created     | client_key: 0x041835D4, datareader_id: 0x000(6), subscriber_id: 0x000(4)

**Without closing this terminal open a new terminal**

**Here type the following**

	ros2 topic list
**You would see something like**

>/cubemx_publisher

>/parameter_events

>/rosout

**If this topic is available then your microros installation is correct and the setup is complete /cubemx_publisher**

# FINAL TEST STEP:
**Run the following line in the second terminal**

	ros2 topic echo /cubemx_publisher
**If you have an output similar to the following, Congrats you have succefully completed microros installation on an STM32 Board**
>---

>data: 0

>---

>data: 1

>---

>data: 2

>--- and so on keeps on increamenting

# Troubleshoot 
>**Note:** If you have an error called as open device error run the following command

	sudo chmod 666 /dev/ttyUSB0



 

# STM32AI Movement Detector
Build a simple movement detector using the AI solutions that are available from STMicroelectronics.

## Description
The goal of this project is to demonstrate how you can use the new ST opensource *AI purpose data logging tools* in your AI based project. We will use the new **AILogging** API to get the data from STM32 to your computer and we will use the **GenericUI** application to display, record and build a dataset from accelerometer data. 

Once the dataset recorded, we will build a simple neural network using **Keras** and we will use the **X-Cube-AI** to convert the Keras model into an optimized C code for STM32.

In short we will cover the data acquisition, the NN model building and the NN model deployment on target.

## Requirements
- An STM32L475 IoTNode from STMicroelectronics: https://www.st.com/en/evaluation-tools/b-l475e-iot01a.html
- The stm32ai-datalogger Git cloned on your computer: https://github.com/STMicroelectronics/stm32ai-datalogger
- STM32CubeIDE to open STM32 Project: https://www.st.com/en/development-tools/stm32cubeide.html
- A Python3.x (Tested with at least 3.6) on your computer with ```pip``` working

## How to do it ?
### Step 1: Record your dataset
To build our dataset we will use a new library released by ST called **AILogging** which is basically a C API that will represents our data as *ai_logging_packet_t* understandable on STM32 and computer side. We will use the C version of this API on STM32 and we will use the GenericUI application which uses python version of this API on computer !
#### Step 1.1: Open the CubeIDE project
Open the CubeIDE project provided and be sure to have the following define ```#define RECORDING_MODE 1``` at the beginning of your project. This will ensure that the project is built for Recording purpose and not including neural network files. 

The project is basically doing:
- Init of USART that will be used to communicate
- Init of LSM6DSL, the accelerometer that we will use to get data
- Init of AILogging API to communicate with the computer
- Get accelerometer's data 
- Build a packet and send it through USART

Compile and Flash the program to the target (and be sure to connect the STM32 on your computer before ;) )


#### Step 1.2: Display your data using GenericUI
From here you have your computer on your desk and the STM32 IoTNode connected to it with the STM32CubeIDE program flashed and running ! 
To check if everything is okay, we can start by displaying data of the accelerometer in realtime on your computer. We are going to perform this task using **GenericUI** provided with the stm32ai-datalogger Git you have should have on your computer.

1) You will have to install Python packages used by **GenericUI**. Simply go into the stm32ai-datalogger/generic_ui folder and run ```pip install -r requirements.txt```
2) You can run the application by typing ```python main.py``` from the *generic_ui* folder
3) If all packages are installed correctly you should see the following screen 
<img width="828" alt="Screenshot 2021-07-05 at 15 29 03" src="https://user-images.githubusercontent.com/5347750/124478716-c470c800-dda5-11eb-8765-552f682222a0.png">

4) The application is widget based, feel free to move, detach, close widgets to make the application your own. You can add new widget by clicking the button in the top bar. Here is the configuration I made: 
<img width="992" alt="Screenshot 2021-07-05 at 15 13 52" src="https://user-images.githubusercontent.com/5347750/124477032-d3567b00-dda3-11eb-9b6f-c88de3a7a8d5.png">

5) Once you have the **GenericUI** application running, it's is time to connect to the target and plot accelerometer's data. You can use the *Connection* widget to select the correct Serial port and click *Connect*. If the port if correct and the application running on the STM32 you should see the following screen:
<img width="1280" alt="Screenshot 2021-07-02 at 16 27 36" src="https://user-images.githubusercontent.com/5347750/124477058-dd787980-dda3-11eb-9992-22cc1eaeed88.png">


#### Step 1.3: Record and export your dataset
Once you have your data that are displayed in **GenericUI** it is time to build the dataset that will be used later to build our NN model. 
To perform this task, be sure to have the *Recorder* widget displayed in **GenericUI**, you can check at the previous picture.
Let's dig into the recording process. The process is done into phases of: labeling, recording, saving. And finally at the end exporting data to file.

We will connect to the STM32 target (you can have *Plot* widget at the same time) and do the following process
- We will record first data for the 'leftright' label
  - Enable the label field in the *Recorder* widget and enter the label 'leftright'
  - Take the STM32 into your hand
  - Click "Start" in the Recorder widget and start shaking your hand from left to right
  - You can "Stop" when you have enough data. For this example we have used around 4400 data for each label
- Once you have stopped you can click on "Save capture" to save the current batch of data into memory. The *data saved* label should display how many data you have currently saved
- Let's do the same for the 'updown' label
  - Be sure to change the label and do the same process of "Start", shaking up and down and "Stop" once you have done
  - Again "Save capture" at the end to save this batch into memory
- Finally let's do a final class labelled 'idle' that will be used when the STM32 is not moving
  - The process will be the same as before
  - You can hold the STM32 into your hand in any position
  - "Save capture" once more time
- You should see the "data saved" label displaying the total number of data that you have recorded so far. 
- Final step, we can export the data into a file just by clicking "Export"

You can get below a visual view of the recording process:
<img width="1280" alt="Screenshot 2021-07-02 at 16 28 53" src="https://user-images.githubusercontent.com/5347750/124480016-1fef8580-dda7-11eb-9e71-bdbc19a928cf.png">

At the end of this step, you should have a CSV file somewhere on your hard drive containing labelled data !

### Step 2: Build your neural network model


### Step 3: Deploy your neural network model on target

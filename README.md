# STM32AI Movement Detector
Build a simple movement detector using the AI solutions that are available from STMicroelectronics.

## Description
The goal of this project is to demonstrate how you can use the new ST opensource *AI purpose data logging tools* in your AI based project. We will use the new **AILogging** API to get the data from STM32 to your computer and we will use the **GenericUI** application to display, record and build a dataset from accelerometer data. 

Once the dataset recorded, we will build a simple neural network using **Keras** and we will use the **X-Cube-AI** to convert the Keras model into an optimized C code for STM32.

In short we will cover the data acquisition, the NN model building and the NN model deployment on target.

## How to do it ?
### Step 1: Record your dataset
To build our dataset we will use a new library released by ST called **AILogging** which is basically a C API that will represents our data as *ai_logging_packet_t* understandable on STM32 and computer side. We will use the C version of this API on STM32 and we will use the GenericUI application which uses python version of this API on computer !
#### Step 1.1: Open the CubeIDE project
#### Step 1.2: Display your data using GenericUI
#### Step 1.3: Record and export your dataset

### Step 2: Build you neural network model

### Step 3: Deploy your neural network model on target

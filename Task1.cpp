//Phase 2
// Task 1

/*
// Execute in .c file
#define CL_TARGET_OPENCL_VERSION 120
#include <CL/cl.h>
#include <stdio.h>

int main() {
    cl_platform_id platform;
    cl_int err;

    // Get the number of available platforms
    cl_uint numPlatforms;
    err = clGetPlatformIDs(0, NULL, &numPlatforms);
    if (err != CL_SUCCESS) {
        fprintf(stderr, "Error getting number of platforms\n");
        return 1;
    }

    // Get platform IDs
    cl_platform_id* platforms = (cl_platform_id*)malloc(numPlatforms * sizeof(cl_platform_id));
    err = clGetPlatformIDs(numPlatforms, platforms, NULL);
    if (err != CL_SUCCESS) {
        fprintf(stderr, "Error getting platform IDs\n");
        free(platforms);
        return 1;
    }

    // Display platform information
    for (cl_uint i = 0; i < numPlatforms; ++i) {
        printf("Platform %d:\n", i + 1);

        // Get platform name
        size_t nameSize;
        err = clGetPlatformInfo(platforms[i], CL_PLATFORM_NAME, 0, NULL, &nameSize);
        if (err != CL_SUCCESS) {
            fprintf(stderr, "Error getting platform name size\n");
            free(platforms);
            return 1;
        }

        char* platformName = (char*)malloc(nameSize);
        err = clGetPlatformInfo(platforms[i], CL_PLATFORM_NAME, nameSize, platformName, NULL);
        if (err != CL_SUCCESS) {
            fprintf(stderr, "Error getting platform name\n");
            free(platforms);
            free(platformName);
            return 1;
        }

        printf("  Name: %s\n", platformName);
        free(platformName);

        // You can retrieve and print other platform information in a similar way
        // For example, CL_PLATFORM_VERSION, CL_PLATFORM_VENDOR, etc.

        printf("\n");
    }

    free(platforms);
    return 0;
}*/


/*
// execute in .cpp file
#include <iostream>
#include <cstdlib>
#include <chrono>

void add_Matrix(float* matrix_1, float* matrix_2, float* result, int rows, int cols) {
    for (int i = 0; i < rows * cols; ++i) {
        result[i] = matrix_1[i] + matrix_2[i];
    }
}

int main() {
    const int rows = 100;
    const int cols = 100;

    // Allocate memory for matrices
    float* matrix_1 = (float*)malloc(rows * cols * sizeof(float));
    float* matrix_2 = (float*)malloc(rows * cols * sizeof(float));
    float* result = (float*)malloc(rows * cols * sizeof(float));

    // Initialize matrices with some values (you can customize this part)
    for (int i = 0; i < rows * cols; ++i) {
        matrix_1[i] = static_cast<float>(i);
        matrix_2[i] = static_cast<float>(i * 2);
    }

    // Start the timer
    auto start_time = std::chrono::high_resolution_clock::now();

    // Perform element-wise addition
    add_Matrix(matrix_1, matrix_2, result, rows, cols);

    // Stop the timer
    auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate the duration
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    // Display the result or perform other operations with the result

    // Display execution time
    std::cout << "Execution Time: " << duration.count() << " microseconds" << std::endl;

    // Free allocated memory
    free(matrix_1);
    free(matrix_2);
    free(result);

    return 0;
}*/


/*
#define CL_TARGET_OPENCL_VERSION 120
#include <CL/cl.h>
#include <iostream>
#include <cstdlib>
#include <Windows.h>

const char* kernelSource = R"(
    __kernel void add_matrix(__global const float* matrix_1,
                             __global const float* matrix_2,
                             __global float* result,
                             const int rows,
                             const int cols)
    {
        int globalID = get_global_id(0);
        int index = globalID * cols;

        for (int i = 0; i < cols; ++i) {
            result[index + i] = matrix_1[index + i] + matrix_2[index + i];
        }
    }
)";

void printDeviceInfo(cl_device_id device) {
    char buffer[1024];
    clGetDeviceInfo(device, CL_DEVICE_NAME, sizeof(buffer), &buffer, NULL);
    std::cout << "Device Name: " << buffer << std::endl;

    clGetDeviceInfo(device, CL_DEVICE_VERSION, sizeof(buffer), &buffer, NULL);
    std::cout << "Device Hardware Version: " << buffer << std::endl;

    clGetDeviceInfo(device, CL_DRIVER_VERSION, sizeof(buffer), &buffer, NULL);
    std::cout << "Device Driver Version: " << buffer << std::endl;

    clGetDeviceInfo(device, CL_DEVICE_OPENCL_C_VERSION, sizeof(buffer), &buffer, NULL);
    std::cout << "Device OpenCL C Version: " << buffer << std::endl;

    cl_uint computeUnits;
    clGetDeviceInfo(device, CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(cl_uint), &computeUnits, NULL);
    std::cout << "Device Parallel Compute Units: " << computeUnits << std::endl;
}

int main() {

    // Get number of platforms
    cl_uint numPlatforms;
    clGetPlatformIDs(0, NULL, &numPlatforms);

    std::cout << "Number of Platforms Detected: " << numPlatforms << std::endl;

    // Get platform information
    cl_platform_id* platforms = (cl_platform_id*)malloc(sizeof(cl_platform_id) * numPlatforms);
    clGetPlatformIDs(numPlatforms, platforms, NULL);

    for (cl_uint i = 0; i < numPlatforms; ++i) {
        char buffer[1024];
        clGetPlatformInfo(platforms[i], CL_PLATFORM_VENDOR, sizeof(buffer), &buffer, NULL);
        std::cout << "Platform Vendor: " << buffer << std::endl;

        clGetPlatformInfo(platforms[i], CL_PLATFORM_NAME, sizeof(buffer), &buffer, NULL);
        std::cout << "Platform Name: " << buffer << std::endl;

        clGetPlatformInfo(platforms[i], CL_PLATFORM_PROFILE, sizeof(buffer), &buffer, NULL);
        std::cout << "Platform Profile: " << buffer << std::endl;

        clGetPlatformInfo(platforms[i], CL_PLATFORM_VERSION, sizeof(buffer), &buffer, NULL);
        std::cout << "Platform Version: " << buffer << std::endl;

        // Get number of devices
        cl_uint numDevices;
        clGetDeviceIDs(platforms[i], CL_DEVICE_TYPE_GPU, 0, NULL, &numDevices);

        std::cout << "Number of Devices Detected: " << numDevices << std::endl;

        // Get device information
        cl_device_id* devices = (cl_device_id*)malloc(sizeof(cl_device_id) * numDevices);
        clGetDeviceIDs(platforms[i], CL_DEVICE_TYPE_GPU, numDevices, devices, NULL);

        for (cl_uint j = 0; j < numDevices; ++j) {
            std::cout << "\nDevice " << j + 1 << ":\n";
            printDeviceInfo(devices[j]);
        }

        free(devices);
    }

    const int rows = 100;
    const int cols = 100;

    // Initialize matrices with some values (you can customize this part)
    float* matrix_1 = (float*)malloc(rows * cols * sizeof(float));
    float* matrix_2 = (float*)malloc(rows * cols * sizeof(float));

    for (int i = 0; i < rows * cols; ++i) {
        matrix_1[i] = static_cast<float>(i);
        matrix_2[i] = static_cast<float>(i * 2);
    }

    // Set up OpenCL context, command queue, and buffers
    cl_platform_id platform;
    clGetPlatformIDs(1, &platform, NULL);

    cl_device_id device;
    clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, NULL);

    cl_context context = clCreateContext(NULL, 1, &device, NULL, NULL, NULL);
    cl_command_queue commandQueue = clCreateCommandQueue(context, device, CL_QUEUE_PROFILING_ENABLE, NULL);

    // Load the kernel source
    cl_program program = clCreateProgramWithSource(context, 1, &kernelSource, NULL, NULL);
    clBuildProgram(program, 1, &device, NULL, NULL, NULL);
    cl_kernel kernel = clCreateKernel(program, "add_matrix", NULL);

    // Create OpenCL buffers
    cl_mem buffer_matrix_1 = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(float) * rows * cols, matrix_1, NULL);
    cl_mem buffer_matrix_2 = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(float) * rows * cols, matrix_2, NULL);
    cl_mem buffer_result = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeof(float) * rows * cols, NULL, NULL);

    // Set kernel arguments
    clSetKernelArg(kernel, 0, sizeof(cl_mem), &buffer_matrix_1);
    clSetKernelArg(kernel, 1, sizeof(cl_mem), &buffer_matrix_2);
    clSetKernelArg(kernel, 2, sizeof(cl_mem), &buffer_result);
    clSetKernelArg(kernel, 3, sizeof(int), &rows);
    clSetKernelArg(kernel, 4, sizeof(int), &cols);

    // Execute the kernel
    size_t globalSize = rows;
    cl_event kernelEvent;
    clEnqueueNDRangeKernel(commandQueue, kernel, 1, NULL, &globalSize, NULL, 0, NULL, &kernelEvent);
    clFinish(commandQueue);

    // Measure kernel execution time
    cl_ulong startTime, endTime;
    clGetEventProfilingInfo(kernelEvent, CL_PROFILING_COMMAND_START, sizeof(cl_ulong), &startTime, NULL);
    clGetEventProfilingInfo(kernelEvent, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), &endTime, NULL);

    // Calculate the duration
    double executionTime = static_cast<double>(endTime - startTime) * 1e-6; // Convert nanoseconds to milliseconds

    // Display execution time
    std::cout << "Kernel Execution Time: " << executionTime << " milliseconds" << std::endl;

    // Read the result from the device to the host
    float* result = (float*)malloc(sizeof(float) * rows * cols);
    clEnqueueReadBuffer(commandQueue, buffer_result, CL_TRUE, 0, sizeof(float) * rows * cols, result, 0, NULL, NULL);

    // Display or use the result matrix as needed

    // Cleanup
    clReleaseMemObject(buffer_matrix_1);
    clReleaseMemObject(buffer_matrix_2);
    clReleaseMemObject(buffer_result);
    clReleaseProgram(program);
    clReleaseKernel(kernel);
    clReleaseCommandQueue(commandQueue);
    clReleaseContext(context);

    free(matrix_1);
    free(matrix_2);
    free(result);
    free(platforms);

    return 0;
}*/
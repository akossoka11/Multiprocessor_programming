/*#define CL_TARGET_OPENCL_VERSION 120
#include <CL/cl.h>
#include "lodepng.h"
#include <iostream>
#include <vector>
#include <chrono>

// OpenCL kernel for resizing the image
const char* resizeImageKernelSource = R"(
    __kernel void resizeImage(__global const uchar4* input, __global uchar4* output, const unsigned int originalWidth, const unsigned int originalHeight) {
        // Calculate new dimensions for the resized image (1/4 of the original size)
        const unsigned int newWidth = originalWidth / 4;
        const unsigned int newHeight = originalHeight / 4;

        // Calculate the global ID
        const unsigned int gidX = get_global_id(0);
        const unsigned int gidY = get_global_id(1);

        // Calculate the indices for the original and resized images
        const unsigned int originalIndex = gidY * originalWidth + gidX;
        const unsigned int resizedIndex = (gidY / 4) * newWidth + (gidX / 4);

        // Copy the pixel from the original to the resized image
        output[resizedIndex] = input[originalIndex];
    }
)";

// OpenCL kernel for converting the image to grayscale
const char* grayScaleImageKernelSource = R"(
    __kernel void grayScaleImage(__global const uchar4* input, __global uchar* output, const unsigned int width, const unsigned int height) {
        // Calculate the global ID
        const unsigned int gidX = get_global_id(0);
        const unsigned int gidY = get_global_id(1);

        // Calculate the index for the input and output images
        const unsigned int index = gidY * width + gidX;

        // Convert RGB to grayscale using the specified formula
        const float3 pixel = convert_float3(input[index].xyz) / 255.0f;
        const float grayValue = 0.2126f * pixel.x + 0.7152f * pixel.y + 0.0722f * pixel.z;

        // Write the grayscale value to the output image
        output[index] = convert_uchar(grayValue * 255.0f);
    }
)";

// OpenCL kernel for applying a 5x5 filter to the image
const char* applyFilterKernelSource = R"(
    __kernel void applyFilter(__global const uchar* input, __global uchar* output, const unsigned int width, const unsigned int height) {
        // Example: Identity filter (no change)
        const float filter[5][5] = {
            {0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0},
            {0, 0, 1, 0, 0},
            {0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0}
        };

        // Calculate the global ID
        const unsigned int gidX = get_global_id(0);
        const unsigned int gidY = get_global_id(1);

        // Skip the border pixels to simplify the example
        if (gidX < 2 || gidY < 2 || gidX >= width - 2 || gidY >= height - 2) {
            return;
        }

        // Apply the filter to each pixel
        float sum = 0.0f;

        for (int i = -2; i <= 2; ++i) {
            for (int j = -2; j <= 2; ++j) {
                const unsigned int index = (gidY + i) * width + (gidX + j);
                sum += convert_float(input[index]) * filter[i + 2][j + 2];
            }
        }

        // Write the filtered value to the output image
        output[gidY * width + gidX] = convert_uchar(sum);
    }
)";

// OpenCL function to profile the execution time of a given kernel
cl_event profileKernel(const cl_command_queue queue, const cl_kernel kernel, const size_t globalSize[2], const size_t localSize[2]) {
    cl_event event;
    clEnqueueNDRangeKernel(queue, kernel, 2, NULL, globalSize, localSize, 0, NULL, &event);
    clFinish(queue);
    return event;
}

// OpenCL function to read image data from a file
std::vector<unsigned char> readImage(const std::string& filename, unsigned& width, unsigned& height) {
    std::vector<unsigned char> image;

    unsigned error = lodepng::decode(image, width, height, filename.c_str());

    if (error) {
        std::cerr << "Error while decoding PNG: " << lodepng_error_text(error) << std::endl;
        return std::vector<unsigned char>();
    }

    return image;
}

// OpenCL function to write image data to a file
void writeImage(const char* filename, const std::vector<unsigned char>& image, unsigned width, unsigned height) {
    unsigned error = lodepng::encode(filename, image, width, height);

    if (error) {
        std::cerr << "Error while encoding PNG: " << lodepng_error_text(error) << std::endl;
        exit(1);
    }
}

int main() {
    // Initialize OpenCL
    cl_platform_id platform;
    clGetPlatformIDs(1, &platform, NULL);

    cl_device_id device;
    clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, NULL);

    cl_context context = clCreateContext(NULL, 1, &device, NULL, NULL, NULL);
    cl_command_queue queue = clCreateCommandQueue(context, device, 0, NULL);

    // Load OpenCL kernels
    cl_program resizeProgram = clCreateProgramWithSource(context, 1, &resizeImageKernelSource, NULL, NULL);
    clBuildProgram(resizeProgram, 1, &device, NULL, NULL, NULL);
    cl_kernel resizeKernel = clCreateKernel(resizeProgram, "resizeImage", NULL);

    cl_program grayScaleProgram = clCreateProgramWithSource(context, 1, &grayScaleImageKernelSource, NULL, NULL);
    clBuildProgram(grayScaleProgram, 1, &device, NULL, NULL, NULL);
    cl_kernel grayScaleKernel = clCreateKernel(grayScaleProgram, "grayScaleImage", NULL);

    cl_program applyFilterProgram = clCreateProgramWithSource(context, 1, &applyFilterKernelSource, NULL, NULL);
    clBuildProgram(applyFilterProgram, 1, &device, NULL, NULL, NULL);
    cl_kernel applyFilterKernel = clCreateKernel(applyFilterProgram, "applyFilter", NULL);

    // Read the original image
    std::string filename = "C:\\Users\\akoss\\MPP\\image_0.png";
    unsigned width, height;
    auto image = readImage(filename, width, height);

    if (image.empty()) {
        return 1;
    }

    // Profile ResizeImage kernel
    cl_mem inputBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(cl_uchar4) * width * height, image.data(), NULL);
    cl_mem outputBuffer = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeof(cl_uchar4) * width / 4 * height / 4, NULL, NULL);
    clSetKernelArg(resizeKernel, 0, sizeof(cl_mem), &inputBuffer);
    clSetKernelArg(resizeKernel, 1, sizeof(cl_mem), &outputBuffer);
    clSetKernelArg(resizeKernel, 2, sizeof(unsigned int), &width);
    clSetKernelArg(resizeKernel, 3, sizeof(unsigned int), &height);
    const size_t resizeGlobalSize[] = { width, height };
    const size_t resizeLocalSize[] = { 16, 16 };
    auto resizeEvent = profileKernel(queue, resizeKernel, resizeGlobalSize, resizeLocalSize);
    clReleaseMemObject(inputBuffer);
    clReleaseMemObject(outputBuffer);

    // Profile GrayScaleImage kernel
    inputBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(cl_uchar4) * width / 4 * height / 4, image.data(), NULL);
    outputBuffer = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeof(cl_uchar) * width / 4 * height / 4, NULL, NULL);
    clSetKernelArg(grayScaleKernel, 0, sizeof(cl_mem), &inputBuffer);
    clSetKernelArg(grayScaleKernel, 1, sizeof(cl_mem), &outputBuffer);
    clSetKernelArg(grayScaleKernel, 2, sizeof(unsigned int), &width);
    clSetKernelArg(grayScaleKernel, 3, sizeof(unsigned int), &height);
    const size_t grayScaleGlobalSize[] = { width / 4, height / 4 };
    const size_t grayScaleLocalSize[] = { 16, 16 };
    auto grayScaleEvent = profileKernel(queue, grayScaleKernel, grayScaleGlobalSize, grayScaleLocalSize);
    clReleaseMemObject(inputBuffer);
    clReleaseMemObject(outputBuffer);

    // Profile ApplyFilter kernel
    inputBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(cl_uchar) * width / 4 * height / 4, image.data(), NULL);
    outputBuffer = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeof(cl_uchar) * width / 4 * height / 4, NULL, NULL);
    clSetKernelArg(applyFilterKernel, 0, sizeof(cl_mem), &inputBuffer);
    clSetKernelArg(applyFilterKernel, 1, sizeof(cl_mem), &outputBuffer);
    clSetKernelArg(applyFilterKernel, 2, sizeof(unsigned int), &width);
    clSetKernelArg(applyFilterKernel, 3, sizeof(unsigned int), &height);
    const size_t applyFilterGlobalSize[] = { width / 4, height / 4 };
    const size_t applyFilterLocalSize[] = { 16, 16 };
    auto applyFilterEvent = profileKernel(queue, applyFilterKernel, applyFilterGlobalSize, applyFilterLocalSize);
    clReleaseMemObject(inputBuffer);
    clReleaseMemObject(outputBuffer);

    // Display profiling information
    clWaitForEvents(1, &resizeEvent);
    cl_ulong resizeTime;
    clGetEventProfilingInfo(resizeEvent, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), &resizeTime, NULL);
    std::cout << "Execution time of ResizeImage: " << resizeTime / 1000000.0 << " ms\n";
    clReleaseEvent(resizeEvent);

    clWaitForEvents(1, &grayScaleEvent);
    cl_ulong grayScaleTime;
    clGetEventProfilingInfo(grayScaleEvent, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), &grayScaleTime, NULL);
    std::cout << "Execution time of GrayScaleImage: " << grayScaleTime / 1000000.0 << " ms\n";
    clReleaseEvent(grayScaleEvent);

    clWaitForEvents(1, &applyFilterEvent);
    cl_ulong applyFilterTime;
    clGetEventProfilingInfo(applyFilterEvent, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), &applyFilterTime, NULL);
    std::cout << "Execution time of ApplyFilter: " << applyFilterTime / 1000000.0 << " ms\n";
    clReleaseEvent(applyFilterEvent);

    // Clean up
    clReleaseKernel(resizeKernel);
    clReleaseKernel(grayScaleKernel);
    clReleaseKernel(applyFilterKernel);
    clReleaseProgram(resizeProgram);
    clReleaseProgram(grayScaleProgram);
    clReleaseProgram(applyFilterProgram);
    clReleaseCommandQueue(queue);
    clReleaseContext(context);

    return 0;
}*/

# add_definitions(-DAPI_EXPORTS)

# FindCUDAToolkit succeeds on JetPack even when nvcc is not in PATH. Pass the
# discovered compiler explicitly before enabling the CUDA language.
if(NOT CMAKE_CUDA_COMPILER)
  if(CUDAToolkit_NVCC_EXECUTABLE)
    set(CMAKE_CUDA_COMPILER "${CUDAToolkit_NVCC_EXECUTABLE}"
        CACHE FILEPATH "CUDA compiler used by vision" FORCE)
  elseif(EXISTS "/usr/local/cuda/bin/nvcc")
    set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc"
        CACHE FILEPATH "CUDA compiler used by vision" FORCE)
  else()
    message(FATAL_ERROR
      "nvcc was not found by CUDAToolkit or at /usr/local/cuda/bin/nvcc")
  endif()
endif()

# Jetson T5000/T4000 (Thor) has CUDA compute capability 11.0. CMake cannot
# detect a default architecture with this JetPack toolchain, so provide the
# target default while preserving any non-empty caller override.
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR
   "${CMAKE_CUDA_ARCHITECTURES}" STREQUAL "")
  set(CMAKE_CUDA_ARCHITECTURES 110 CACHE STRING
      "CUDA architectures used to build vision kernels" FORCE)
endif()
enable_language(CUDA)

if(TENSORRT_MAJOR_VERSION VERSION_GREATER_EQUAL 10)
  # JetPack 7.2 ships TensorRT 10.  The supplied .engine is already
  # serialized, so only the runtime, preprocessing, and inference code are
  # needed.  The remaining files implement the removed TensorRT 8.6 plugin
  # and engine-builder APIs.
  set(SRCS impl.cpp preprocess.cu)
elseif(TENSORRT_MAJOR_VERSION VERSION_EQUAL 8)
  set(SRCS block.cpp
           calibrator.cpp
           impl.cpp
           model.cpp
           postprocess.cpp
           postprocess.cu
           preprocess.cu
           yololayer.cu)
else()
  message(FATAL_ERROR
    "Unsupported TensorRT major version: ${TENSORRT_MAJOR_VERSION}; "
    "vision supports TensorRT 8.x and 10.x")
endif()

add_library(yolov8_trt SHARED ${SRCS})
target_link_libraries(yolov8_trt PRIVATE ${TENSORRT_LIB}
                                         CUDA::cudart
                                 PUBLIC ${OpenCV_LIBS})
# target_compile_options(yolov8_trt PRIVATE -w -Wno-dev)
target_compile_options(yolov8_trt PRIVATE -w)
install(TARGETS yolov8_trt DESTINATION lib)
                              
if(TENSORRT_MAJOR_VERSION VERSION_LESS 10)
  add_executable(yolov8_det yolov8_det.cpp)
  target_link_libraries(yolov8_det yolov8_trt ${TENSORRT_LIB} CUDA::cudart)
  target_compile_options(yolov8_det PRIVATE -w)

  add_executable(yolov8_seg yolov8_seg.cpp)
  target_link_libraries(yolov8_seg yolov8_trt ${TENSORRT_LIB} CUDA::cudart)
  target_compile_options(yolov8_seg PRIVATE -w)

  install(TARGETS yolov8_det yolov8_seg DESTINATION lib/${PROJECT_NAME})
endif()

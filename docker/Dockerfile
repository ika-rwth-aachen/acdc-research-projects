# docker buildx build --push --platform linux/amd64,linux/arm64 -t rwthika/acdc-research-projects:latest .

FROM rwthika/acdc-notebooks:latest as base

USER root

# enable sudo for user
ENV DOCKERUSER jovyan
RUN sed -i "s/#%sudo/%sudo/" /etc/sudoers && \
    usermod -aG sudo $DOCKERUSER && \
    touch /home/$DOCKERUSER/.sudo_as_admin_successful && \
    echo "$DOCKERUSER:$DOCKERUSER" | chpasswd

FROM base as final-amd64

# install CUDA for GPU support (copy from official cuda Dockerfiles)
# ==============================================================================

# https://gitlab.com/nvidia/container-images/cuda/-/blob/master/dist/11.2.1/ubuntu2004/base/Dockerfile
ENV NVARCH x86_64
ENV NVIDIA_REQUIRE_CUDA "cuda>=11.2 brand=tesla,driver>=418,driver<419 brand=tesla,driver>=450,driver<451"
ENV NV_CUDA_CUDART_VERSION 11.2.146-1
ENV NV_CUDA_COMPAT_PACKAGE cuda-compat-11-2
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg2 curl ca-certificates && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/${NVARCH}/3bf863cc.pub | apt-key add - && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/${NVARCH} /" > /etc/apt/sources.list.d/cuda.list && \
    apt-get purge --autoremove -y curl \
    && rm -rf /var/lib/apt/lists/*
ENV CUDA_VERSION 11.2.1
# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-cudart-11-2=${NV_CUDA_CUDART_VERSION} \
    ${NV_CUDA_COMPAT_PACKAGE} \
    && rm -rf /var/lib/apt/lists/*
# Required for nvidia-docker v1
RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf \
    && echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf
ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local//usr/local/nvidia/lib:/usr/local/nvidia/lib64
# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility

# https://gitlab.com/nvidia/container-images/cuda/-/blob/master/dist/11.2.1/ubuntu2004/runtime/Dockerfile
ENV NV_CUDA_LIB_VERSION 11.2.1-1
ENV NV_NVTX_VERSION 11.2.67-1
ENV NV_LIBNPP_VERSION 11.3.2.139-1
ENV NV_LIBNPP_PACKAGE libnpp-11-2=${NV_LIBNPP_VERSION}
ENV NV_LIBCUSPARSE_VERSION 11.4.0.135-1
ENV NV_LIBCUBLAS_PACKAGE_NAME libcublas-11-2
ENV NV_LIBCUBLAS_VERSION 11.4.1.1026-1
ENV NV_LIBCUBLAS_PACKAGE ${NV_LIBCUBLAS_PACKAGE_NAME}=${NV_LIBCUBLAS_VERSION}
ENV NV_LIBNCCL_PACKAGE_NAME libnccl2
ENV NV_LIBNCCL_PACKAGE_VERSION 2.8.4-1
ENV NCCL_VERSION 2.8.4-1
ENV NV_LIBNCCL_PACKAGE ${NV_LIBNCCL_PACKAGE_NAME}=${NV_LIBNCCL_PACKAGE_VERSION}+cuda11.2
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-libraries-11-2=${NV_CUDA_LIB_VERSION} \
    ${NV_LIBNPP_PACKAGE} \
    cuda-nvtx-11-2=${NV_NVTX_VERSION} \
    libcusparse-11-2=${NV_LIBCUSPARSE_VERSION} \
    ${NV_LIBCUBLAS_PACKAGE} \
    ${NV_LIBNCCL_PACKAGE} \
    && rm -rf /var/lib/apt/lists/*
# Keep apt from auto upgrading the cublas and nccl packages. See https://gitlab.com/nvidia/container-images/cuda/-/issues/88
RUN apt-mark hold ${NV_LIBCUBLAS_PACKAGE_NAME} ${NV_LIBNCCL_PACKAGE_NAME}
# Add entrypoint items
ENV NVIDIA_PRODUCT_NAME="CUDA"
ENV NVIDIA_CUDA_END_OF_LIFE=1

# https://gitlab.com/nvidia/container-images/cuda/-/blob/master/dist/11.2.1/ubuntu2004/runtime/cudnn8/Dockerfile
ENV NV_CUDNN_VERSION 8.1.1.33
ENV NV_CUDNN_PACKAGE_NAME "libcudnn8"
ENV NV_CUDNN_PACKAGE "libcudnn8=$NV_CUDNN_VERSION-1+cuda11.2"
RUN apt-get update && apt-get install -y --no-install-recommends \
    ${NV_CUDNN_PACKAGE} \
    && apt-mark hold ${NV_CUDNN_PACKAGE_NAME} \
    && rm -rf /var/lib/apt/lists/*

# ==============================================================================

ENV LD_LIBRARY_PATH=/usr/local/cuda-11.2/targets/x86_64-linux/lib:$LD_LIBRARY_PATH

FROM base as final-arm64

FROM final-${TARGETARCH} as final

USER jovyan
FROM rwthika/acdc-notebooks:latest


USER root

# enable sudo for user
ENV DOCKERUSER jovyan
RUN sed -i "s/#%sudo/%sudo/" /etc/sudoers && \
    usermod -aG sudo $DOCKERUSER && \
    touch /home/$DOCKERUSER/.sudo_as_admin_successful && \
    echo "$DOCKERUSER:$DOCKERUSER" | chpasswd

USER jovyan

# install CUDA for GPU support
RUN conda install -y cudatoolkit-dev=11.2 cudnn=8.1.0
RUN echo 'export LD_PRELOAD=$CONDA_PREFIX/lib/libcudart.so:$CONDA_PREFIX/lib/libcublas.so:$CONDA_PREFIX/lib/libcublasLt.so:$CONDA_PREFIX/lib/libcufft.so:$CONDA_PREFIX/lib/libcurand.so:$CONDA_PREFIX/lib/libcusolver.so:$CONDA_PREFIX/lib/libcusparse.so:$CONDA_PREFIX/lib/libcudnn.so' >> ~/.bashrc

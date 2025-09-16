# Dockerfile (Versión Definitiva con sudo corregido)

# --- 1. IMAGEN BASE ---
FROM osrf/ros:humble-desktop

# --- 2. CONFIGURACIÓN DEL ENTORNO ---
SHELL ["/bin/bash", "-c"]

# --- 3. INSTALACIÓN DE DEPENDENCIAS ---
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    git \
    wget \
    xterm \
    sudo \
    python3-pip \
    ros-humble-turtlebot3* \
    ros-humble-xacro && \
    rm -rf /var/lib/apt/lists/*

# --- 4. CREAR UN USUARIO NO-ROOT ---
RUN useradd -m user && \
    echo "user:user" | chpasswd && \
    adduser user sudo

# --- 5. DESCARGAR RECURSOS EXTERNOS ---
WORKDIR /
#RUN git clone https://github.com/wsnewman/gazebo_models_worlds_collection.git

# --- 6. CONFIGURAR VARIABLES DE ENTORNO ---
#ENV GZ_SIM_RESOURCE_PATH=/gazebo_models_worlds_collection
#ENV MESA_GL_VERSION_OVERRIDE=4.1

# --- 7. CREAR Y PREPARAR EL WORKSPACE ---
WORKDIR /ros2_ws
RUN chown -R user:user /ros2_ws
COPY --chown=user:user ./ros2_ws/src ./src

# --- 8. CAMBIAR AL USUARIO NO-ROOT ---
USER user
WORKDIR /ros2_ws

# --- 9. CONSTRUIR EL WORKSPACE ---
RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# --- 10. CONFIGURAR EL PUNTO DE ENTRADA (ENTRYPOINT) ---
COPY --chown=user:user ./entrypoint.sh /home/user/
# <-- ¡LA CORRECCIÓN ESTÁ AQUÍ! Hemos quitado "sudo".
RUN chmod +x /home/user/entrypoint.sh
ENTRYPOINT ["/home/user/entrypoint.sh"]

# --- 11. COMANDO POR DEFECTO: Iniciar una terminal ---
CMD ["bash"]

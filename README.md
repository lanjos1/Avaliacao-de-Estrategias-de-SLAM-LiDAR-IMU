# Avaliacao-de-Estrategias-de-SLAM-LiDAR-IMU-
Análise comparativa de estratégias SLAM LiDAR-IMU com LeGO-LOAM e LIO-SAM em ambiente simulado ROS/Gazebo, utilizando o UGV Clearpath Husky A200 e sensor Velodyne Puck VLP-16. Avalia acoplamento fraco (LeGO-LOAM) e fortemente acoplado (LIO-SAM) com dados de nuvem de pontos e IMU.


A bag com o caminho do husky -> https://drive.google.com/drive/folders/1uE2QlnkfLwvhhe4cZrvPHQz-vqMgg5U5?usp=sharing



## Resultados

### Precisão de Trajetória

| Algoritmo / Modo | Erro Médio Pos (m) | Erro Máximo Pos (m) | Erro Médio Yaw (°) |
|:---|:---:|:---:|:---:|
| LeGO-LOAM (Com IMU) | 0.0632 | 0.1549 | 94.8253 |
| LeGO-LOAM (Sem IMU) | 0.0680 | 0.1620 | 94.7035 |
| LIO-SAM | 0.1470 | 0.2449 | 2.0562 |

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

### Eficiência Computacional e Taxa de Atualização

| Algoritmo / Modo | Frequência Média (Hz) | Tempo Médio (ms) | Tempo Máximo (ms) | Jitter / Std (ms) |
|:---|:---:|:---:|:---:|:---:|
| LeGO-LOAM (Com IMU) | 10.00 | 100.00 | 107.11 | 2.87 |
| LeGO-LOAM (Sem IMU) | 9.88 | 101.21 | 110.68 | 3.99 |
| LIO-SAM | 5.01 | 199.40 | 220.82 | 6.69 |

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def carregar_posicoes(filename):
    df = pd.read_csv(filename).dropna()
    col_time = [c for c in df.columns if 'time' in c][0]
    col_x = [c for c in df.columns if 'position.x' in c][0]
    col_y = [c for c in df.columns if 'position.y' in c][0]
    col_z = [c for c in df.columns if 'position.z' in c][0]
    
    raw_time = df[col_time].values
    if raw_time[0] > 1e11:
        t = raw_time / 1e9
    else:
        t = raw_time
        
    t = t - t[0]
    pos = df[[col_x, col_y, col_z]].values
    return t, pos

def calcular_transformacao_rigid_se3(t_slam, pos_slam, t_gt, pos_gt):

    pos_gt_sinc = []
    for t_s in t_slam:
        idx_gt = np.argmin(np.abs(t_gt - t_s))
        pos_gt_sinc.append(pos_gt[idx_gt])
    pos_gt_sinc = np.array(pos_gt_sinc)
    
    mu_X = np.mean(pos_slam, axis=0)
    mu_Y = np.mean(pos_gt_sinc, axis=0)
    X_c = pos_slam - mu_X
    Y_c = pos_gt_sinc - mu_Y
    
    C = np.dot(Y_c.T, X_c) / len(pos_slam)
    U, S, Vt = np.linalg.svd(C)
    
    R = np.dot(U, Vt)
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = np.dot(U, Vt)
        
    t_vec = mu_Y - np.dot(R, mu_X)
    return R, t_vec

def alinhar_liosam_direto(pos_lio, pos_gt):
    lio_local = pos_lio - pos_lio[0]
    return lio_local + pos_gt[0]

try:
    print("A carregar os ficheiros de trajetórias...")
    t_gt, pos_gt = carregar_posicoes('ground_truth.csv')
    t_com, pos_com = carregar_posicoes('lego_loam_com_imu_68.csv')
    t_sem, pos_sem = carregar_posicoes('lego_loam_sem_imu_68.csv')
    t_lio, pos_lio = carregar_posicoes('lio_sam.csv')

    print("A processar alinhamentos espaciais...")
    
    R_com, t_com_trans = calcular_transformacao_rigid_se3(t_com, pos_com, t_gt, pos_gt)
    R_sem, t_sem_trans = calcular_transformacao_rigid_se3(t_sem, pos_sem, t_gt, pos_gt)
    
    pos_com_alinhada = np.dot(pos_com, R_com.T) + t_com_trans
    pos_sem_alinhada = np.dot(pos_sem, R_sem.T) + t_sem_trans
    
    pos_lio_alinhada = alinhar_liosam_direto(pos_lio, pos_gt)

    plt.figure(figsize=(10, 9))
    
    plt.plot(pos_gt[:, 0], pos_gt[:, 1], label='Ground Truth (Gazebo)', color='black', linewidth=2.5, linestyle='--')
    plt.plot(pos_com_alinhada[:, 0], pos_com_alinhada[:, 1], label='LeGO-LOAM (Com IMU)', color='blue', linewidth=2)
    plt.plot(pos_sem_alinhada[:, 0], pos_sem_alinhada[:, 1], label='LeGO-LOAM (Sem IMU)', color='red', linewidth=1.5, linestyle=':')
    plt.plot(pos_lio_alinhada[:, 0], pos_lio_alinhada[:, 1], label='LIO-SAM', color='darkgreen', linewidth=2, linestyle='-')
    
    plt.title('Comparação das Trajetórias')
    plt.xlabel('Posição X (metros)')
    plt.ylabel('Posição Y (metros)')
    plt.legend(loc='best')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.axis('equal')  
    
    plt.tight_layout()
    plt.savefig('comparacao_trajetorias_xy.png', dpi=300)
    print("\n[SUCESSO] Gráfico final gerado!")

except Exception as e:
    print(f"Erro ao processar e alinhar trajetórias: {e}")
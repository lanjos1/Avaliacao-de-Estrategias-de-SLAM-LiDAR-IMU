import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def carregar_e_limpar_csv(filename):
    df = pd.read_csv(filename).dropna()
    col_time = [c for c in df.columns if 'time' in c][0]
    col_x = [c for c in df.columns if 'position.x' in c][0]
    col_y = [c for c in df.columns if 'position.y' in c][0]
    col_z = [c for c in df.columns if 'position.z' in c][0]
    
    col_qx = [c for c in df.columns if 'orientation.x' in c][0]
    col_qy = [c for c in df.columns if 'orientation.y' in c][0]
    col_qz = [c for c in df.columns if 'orientation.z' in c][0]
    col_qw = [c for c in df.columns if 'orientation.w' in c][0]
    
    raw_time = df[col_time].values
    if raw_time[0] > 1e11:
        t = raw_time / 1e9  
    else:
        t = raw_time
        
    t = t - t[0]
    pos = df[[col_x, col_y, col_z]].values
    
    qx, qy, qz, qw = df[col_qx].values, df[col_qy].values, df[col_qz].values, df[col_qw].values
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    
    return t, pos, yaw

def carregar_liosam_original(filename):
    df = pd.read_csv(filename).dropna()
    
    t_col = [c for c in df.columns if 'time' in c][0]
    time_lio = df[t_col].to_numpy()
    time_lio = time_lio - time_lio[0]
    
    col_x = [c for c in df.columns if 'position.x' in c][0]
    col_y = [c for c in df.columns if 'position.y' in c][0]
    col_z = [c for c in df.columns if 'position.z' in c][0]
    pos = df[[col_x, col_y, col_z]].values
    
    try:
        col_qx = [c for c in df.columns if 'orientation.x' in c][0]
        col_qy = [c for c in df.columns if 'orientation.y' in c][0]
        col_qz = [c for c in df.columns if 'orientation.z' in c][0]
        col_qw = [c for c in df.columns if 'orientation.w' in c][0]
        qx, qy, qz, qw = df[col_qx].values, df[col_qy].values, df[col_qz].values, df[col_qw].values
        yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    except Exception:
        yaw = np.zeros(len(time_lio))
        
    return time_lio, pos, yaw

def calcular_umeyama_se3(pos_slam, pos_gt_sinc):
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

try:
    print("A carregar trajetórias e referências dos ficheiros...")
    t_gt, pos_gt, yaw_gt = carregar_e_limpar_csv('ground_truth.csv')
    t_com, pos_com, yaw_com = carregar_e_limpar_csv('lego_loam_com_imu_68.csv')
    t_sem, pos_sem, yaw_sem = carregar_e_limpar_csv('lego_loam_sem_imu_68.csv')
    t_lio, pos_lio, yaw_lio = carregar_liosam_original('lio_sam.csv')

    print("A processar métricas do LeGO-LOAM (Com IMU)...")
    pos_gt_sinc_com = np.array([pos_gt[np.argmin(np.abs(t_gt - t_s))] for t_s in t_com])
    yaw_gt_sinc_com = np.array([yaw_gt[np.argmin(np.abs(t_gt - t_s))] for t_s in t_com])
    R_com, t_com_trans = calcular_umeyama_se3(pos_com, pos_gt_sinc_com)
    pos_com_alinhada = np.dot(pos_com, R_com.T) + t_com_trans
    err_pos_com = np.linalg.norm(pos_com_alinhada - pos_gt_sinc_com, axis=1)
    diff_yaw_com = (yaw_com + np.arctan2(R_com[1,0], R_com[0,0]) - yaw_gt_sinc_com + np.pi) % (2*np.pi) - np.pi
    err_yaw_com_deg = np.abs(np.degrees(diff_yaw_com))

    print("A processar métricas do LeGO-LOAM (Sem IMU)...")
    pos_gt_sinc_sem = np.array([pos_gt[np.argmin(np.abs(t_gt - t_s))] for t_s in t_sem])
    yaw_gt_sinc_sem = np.array([yaw_gt[np.argmin(np.abs(t_gt - t_s))] for t_s in t_sem])
    R_sem, t_sem_trans = calcular_umeyama_se3(pos_sem, pos_gt_sinc_sem)
    pos_sem_alinhada = np.dot(pos_sem, R_sem.T) + t_sem_trans
    err_pos_sem = np.linalg.norm(pos_sem_alinhada - pos_gt_sinc_sem, axis=1)
    diff_yaw_sem = (yaw_sem + np.arctan2(R_sem[1,0], R_sem[0,0]) - yaw_gt_sinc_sem + np.pi) % (2*np.pi) - np.pi
    err_yaw_sem_deg = np.abs(np.degrees(diff_yaw_sem))

    print("A aplicar reconstrução matemática fiel do LIO-SAM (Interpolação + Centroide)...")
    gt_x_interp = np.interp(t_lio, t_gt, pos_gt[:, 0])
    gt_y_interp = np.interp(t_lio, t_gt, pos_gt[:, 1])
    gt_z_interp = np.interp(t_lio, t_gt, pos_gt[:, 2])
    pos_gt_sinc_lio = np.column_stack((gt_x_interp, gt_y_interp, gt_z_interp))
    yaw_gt_sinc_lio = np.interp(t_lio, t_gt, yaw_gt)

    centro_gt_lio = np.mean(pos_gt_sinc_lio, axis=0)
    centro_lio = np.mean(pos_lio, axis=0)
    gt_alinhado_lio = pos_gt_sinc_lio - centro_gt_lio
    lio_alinhado_puro = pos_lio - centro_lio

    err_pos_lio = np.sqrt((gt_alinhado_lio[:, 0] - lio_alinhado_puro[:, 0])**2 + 
                          (gt_alinhado_lio[:, 1] - lio_alinhado_puro[:, 1])**2)
    diff_yaw_lio = (yaw_lio - yaw_gt_sinc_lio + np.pi) % (2 * np.pi) - np.pi
    err_yaw_lio_deg = np.abs(np.degrees(diff_yaw_lio))

    print("\n" + "="*85)
    print("TABELA COMPARATIVA")
    print("="*85)
    print("| Algoritmo / Modo | Erro Médio Pos (m) | Erro Máximo Pos (m) | Erro Médio Yaw (°) |")
    print("| :--- | :---: | :---: | :---: |")
    print(f"| LeGO-LOAM (Com IMU) | {np.mean(err_pos_com):.4f} | {np.max(err_pos_com):.4f} | {np.mean(err_yaw_com_deg):.4f} |")
    print(f"| LeGO-LOAM (Sem IMU) | {np.mean(err_pos_sem):.4f} | {np.max(err_pos_sem):.4f} | {np.mean(err_yaw_sem_deg):.4f} |")
    print(f"| LIO-SAM              | {np.mean(err_pos_lio):.4f} | {np.max(err_pos_lio):.4f} | {np.mean(err_yaw_lio_deg):.4f} |")
    print("="*85)

    plt.figure(figsize=(14, 6))
    
    plt.subplot(1, 2, 1)
    plt.plot(t_com, err_pos_com, color='blue', label='LeGO-LOAM (Com IMU)', linewidth=2)
    plt.plot(t_sem, err_pos_sem, color='red', label='LeGO-LOAM (Sem IMU)', linewidth=1.5, linestyle=':')
    plt.plot(t_lio, err_pos_lio, color='darkgreen', label='LIO-SAM', linewidth=2, linestyle='-')
    plt.title('Evolução Temporal do Erro de Posição')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Erro de Posição (metros)')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.legend()
    
    plt.subplot(1, 2, 2)
    plt.plot(t_com, err_yaw_com_deg, color='blue', label='LeGO-LOAM (Com IMU)', linewidth=2)
    plt.plot(t_sem, err_yaw_sem_deg, color='red', label='LeGO-LOAM (Sem IMU)', linewidth=1.5, linestyle=':')
    plt.plot(t_lio, err_yaw_lio_deg, color='darkgreen', label='LIO-SAM', linewidth=2, linestyle='-')
    plt.title('Evolução Temporal do Erro de Orientação (Yaw)')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Erro Absoluto (graus)')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.legend()
    
    plt.tight_layout()
    plt.savefig('grafico_comparativo_erros_unificados.png', dpi=300)
    print("\n[SUCESSO] Gráfico 'grafico_comparativo_erros_unificados.png' guardado!")

except Exception as e:
    print(f"Erro ao executar processamento global: {e}")
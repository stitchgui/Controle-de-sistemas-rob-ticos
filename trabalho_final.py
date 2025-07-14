import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
from spatialmath import SE3
import matplotlib.pyplot as plt

L1 = 0.5
L2 = 0.3
H = 0.0

# Parâmetros dinâmicos para uma simulação física
massa_elos = [1.5, 1.0, 0.5]
centro_de_massa_elos = np.array([
    [L1/2, 0, 0],
    [L2/2, 0, 0],
    [0, 0, 0]
]).T

robot = rtb.DHRobot([
        rtb.RevoluteDH(a=L1, d=H, alpha=np.pi, m=massa_elos[0], r=centro_de_massa_elos[:,0]),
        rtb.RevoluteDH(a=L2, d=0, m=massa_elos[1], r=centro_de_massa_elos[:,1]),
        rtb.PrismaticDH(theta=0, a=0, qlim=[0, 0.2], m=massa_elos[2], r=[0,0,0]) # Ajustado 'r' para ser uma lista de 3 elementos para a junta prismática
    ], name="SCARA com Dinâmica")

print("Modelo do Robô:")
print(robot)


def resolver_cinematica_inversa(x, y, z):
    distancia_xy = np.sqrt(x**2 + y**2)
    if not (abs(L1 - L2) <= distancia_xy <= (L1 + L2)):
        return None
    q3 = -z
    cos_q2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(cos_q2) > 1:
        cos_q2 = np.sign(cos_q2)
    q2 = -np.arccos(cos_q2) # Escolhendo a solução "elbow down"
    k1 = L1 + L2 * np.cos(q2)
    k2 = L2 * np.sin(q2)
    q1 = np.arctan2(y, x) - np.arctan2(k2, k1)
    return np.array([q1, q2, q3])


q_home = np.array([0, -np.pi / 2, 0])
pontos_cartesianos = {
    "P_pegar":   [0.6, 0.0, -0.15],
    "P_colocar": [0.0, 0.5, -0.20]
}
pontos_cartesianos["P_pegar_acima"] = pontos_cartesianos["P_pegar"] + np.array([0, 0, 0.1])
pontos_cartesianos["P_colocar_acima"] = pontos_cartesianos["P_colocar"] + np.array([0, 0, 0.15])

configs_de_junta = {"P_home": q_home}
ordem_tarefa = ["P_pegar_acima", "P_pegar", "P_colocar_acima", "P_colocar"]
for nome in ordem_tarefa:
    ponto = pontos_cartesianos[nome]
    q_solucao = resolver_cinematica_inversa(ponto[0], ponto[1], ponto[2])
    if q_solucao is not None:
        configs_de_junta[nome] = q_solucao
    else:
        raise ValueError(f"ERRO CRÍTICO: O ponto {nome} está fora do alcance do robô.")

q_home = configs_de_junta["P_home"]
q_pegar_acima = configs_de_junta["P_pegar_acima"]
q_pegar = configs_de_junta["P_pegar"]
q_colocar_acima = configs_de_junta["P_colocar_acima"]
q_colocar = configs_de_junta["P_colocar"]
t_horizontal = np.linspace(0, 1.5, 150)
t_vertical = np.linspace(0, 0.5, 50)
traj1 = rtb.jtraj(q_home, q_pegar_acima, t_horizontal)
traj2 = rtb.jtraj(q_pegar_acima, q_pegar, t_vertical)
traj3 = rtb.jtraj(q_pegar, q_pegar_acima, t_vertical)
traj4 = rtb.jtraj(q_pegar_acima, q_colocar_acima, t_horizontal)
traj5 = rtb.jtraj(q_colocar_acima, q_colocar, t_vertical)
traj6 = rtb.jtraj(q_colocar, q_colocar_acima, t_vertical)
traj7 = rtb.jtraj(q_colocar_acima, q_home, t_horizontal)
q_desejado = np.vstack([traj1.q, traj2.q, traj3.q, traj4.q, traj5.q, traj6.q, traj7.q])
qd_desejado = np.vstack([traj1.qd, traj2.qd, traj3.qd, traj4.qd, traj5.qd, traj6.qd, traj7.qd])
print(f"\nTrajetória completa gerada com {q_desejado.shape[0]} pontos.")

def analisar_singularidade(q_configuracao_atual, robo_modelo):
    """
    Analisa se uma dada configuração de juntas (q) do robô está em uma singularidade
    e retorna o Jacobiano completo e o determinante da sub-matriz linear (3x3).

    Args:
        q_configuracao_atual (np.array): Um array NumPy contendo as posições das juntas.
        robo_modelo (rtb.DHRobot): O objeto robô definido pelo roboticstoolbox.

    Returns:
        tuple: (bool, np.array, float, int), onde o booleano indica se é singular (True/False),
               o np.array é o Jacobiano completo, o float é o determinante da sub-matriz linear (3x3)
               e o int é o posto do Jacobiano.
    """
    if len(q_configuracao_atual) != robo_modelo.n:
        raise ValueError(f"A configuração das juntas deve ter {robo_modelo.n} elementos.")

    J = robo_modelo.jacob0(q_configuracao_atual) # Jacobiano completo (6x3 para um robô de 3 DOFs)
    rank_J = np.linalg.matrix_rank(J)
    is_singular = rank_J < robo_modelo.n

    J_linear = J[:3, :]

    det_J_linear = np.linalg.det(J_linear)

    return is_singular, J, det_J_linear, rank_J

print("\n--- Iniciando Simulação do Controlador PID com Análise de Singularidade ---")
dt = 0.01
num_passos = q_desejado.shape[0]
t_simulacao = np.arange(0, num_passos * dt, dt)
Kp = np.diag([250, 150, 5000])
Ki = np.diag([30, 30, 35])
Kd = np.diag([10, 10, 50])

q_hist = np.zeros((num_passos, robot.n))
tau_hist = np.zeros((num_passos, robot.n))
erro_hist = np.zeros((num_passos, robot.n))
singularidade_hist = np.zeros(num_passos, dtype=bool) # Para registrar singularidades baseadas no rank
jac_det_hist = np.zeros(num_passos) # Para registrar o determinante do Jacobiano

q_atual = q_desejado[0, :]
qd_atual = np.zeros(robot.n)
erro_integral = np.zeros(robot.n)

for i in range(num_passos):
    q_ref = q_desejado[i, :]
    qd_ref = qd_desejado[i, :]

    # --- Análise de Singularidade no Ponto Atual ---
    is_singular, J_completo, det_jac, rank_jac = analisar_singularidade(q_atual, robot)
    singularidade_hist[i] = is_singular
    jac_det_hist[i] = det_jac

    if is_singular:
        print(f"AVISO: Singularidade detectada em t={t_simulacao[i]:.2f}s "
              f"na configuração q={np.round(np.rad2deg(q_atual[:2]), 2)} deg, {np.round(q_atual[2], 3)} m "
              f"(det(J_linear)={det_jac:.2e}, rank={rank_jac})")
    
    erro = q_ref - q_atual
    erro_derivada = qd_ref - qd_atual
    erro_integral += erro * dt
    tau = Kp @ erro + Ki @ erro_integral + Kd @ erro_derivada
    
    # --- Cálculo Manual da Dinâmica ---
    M = robot.inertia(q_atual)
    C_matrix = robot.coriolis(q_atual, qd_atual)
    C_term = C_matrix @ qd_atual
    
    g = robot.gravload(q_atual)
    
    tau_net = tau - C_term - g
    
    qdd_atual = np.linalg.solve(M, tau_net)

    # Integração Numérica
    qd_atual += qdd_atual * dt
    q_atual += qd_atual * dt
    
    q_hist[i, :] = q_atual
    tau_hist[i, :] = tau
    erro_hist[i, :] = erro

print("Simulação concluída com sucesso.")

num_singular_points = np.sum(singularidade_hist)
percent_singular = (num_singular_points / num_passos) * 100 if num_passos > 0 else 0
print(f"\n--- Resumo da Análise de Singularidade ---")
print(f"Total de pontos da trajetória simulados: {num_passos}")
print(f"Número de pontos singulares detectados: {num_singular_points}")
print(f"Porcentagem da trajetória em singularidade: {percent_singular:.2f}%")


print("Gerando gráficos de análise...")
plt.style.use('seaborn-v0_8-whitegrid')
for j in range(robot.n):
    plt.figure(figsize=(18, 5))
    plt.suptitle(f'Análise de Desempenho para a Junta {j+1}', fontsize=16)

    # Gráfico de Seguimento da Trajetória
    plt.subplot(1, 3, 1)
    plt.plot(t_simulacao, q_desejado[:, j], 'r--', label='Posição Desejada')
    plt.plot(t_simulacao, q_hist[:, j], 'b', label='Posição Obtida')
    
    # Adicionanando marcadores para singularidades no gráfico de trajetória (baseado no rank)
    singular_times_rank = t_simulacao[singularidade_hist]
    singular_q_hist_rank = q_hist[singularidade_hist, j]
    if len(singular_times_rank) > 0:
        plt.plot(singular_times_rank, singular_q_hist_rank, 'ro', markersize=4, label='Singularidade (Rank)', alpha=0.6)

    plt.title('Seguimento da Trajetória')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Posição (rad)' if j < 2 else 'Posição (m)')
    plt.legend()
    plt.grid(True)
    
    # Gráfico de Evolução do Erro
    plt.subplot(1, 3, 2)
    plt.plot(t_simulacao, erro_hist[:, j], 'g')
    plt.title('Evolução do Erro')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Erro (rad)' if j < 2 else 'Erro (m)')
    plt.grid(True)
    
    # Gráfico de Sinal de Controle
    plt.subplot(1, 3, 3)
    plt.plot(t_simulacao, tau_hist[:, j], 'k')
    plt.title('Sinal de Controle')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Torque (Nm)' if j < 2 else 'Força (N)')
    plt.grid(True)
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()
    
#Determinante do Jacobiano ao longo da trajetória
plt.figure(figsize=(10, 5))
plt.plot(t_simulacao, np.abs(jac_det_hist), 'm-', label='|Determinante do Jacobiano Linear|')
singular_times_det = t_simulacao[singularidade_hist]
if len(singular_times_det) > 0:
    # Usar scatter para marcar pontos, pois pode haver vários pontos singulares agrupados
    plt.scatter(singular_times_det, np.abs(jac_det_hist[singularidade_hist]), 
                color='red', marker='x', s=50, zorder=5, label='Ponto Singular (Rank)')

plt.title('Módulo do Determinante do Jacobiano Linear (3x3) ao longo da Trajetória')
plt.xlabel('Tempo (s)')
plt.ylabel('|det(J_linear)|')
plt.yscale('log')
plt.legend()
plt.grid(True, which="both", ls="-")
plt.tight_layout()
plt.show()

print("\nGerando animação GIF do robô...")
try:
    robot.plot(
        q_hist,
        dt=dt,
        backend='pyplot',
        movie='trajetoria_robo_final.gif',
        limits=[-0.8, 0.8, -0.8, 0.8, -0.25, 0.25] # [xmin, xmax, ymin, ymax, zmin, zmax]
    )
    print("GIF 'trajetoria_robo_final.gif' gerado com sucesso!")

except Exception as e:
    print(f"\nOcorreu um erro ao gerar a animação: {e}")
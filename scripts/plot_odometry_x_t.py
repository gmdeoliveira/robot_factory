import csv
import matplotlib.pyplot as plt 

# Função para carregar os dados do arquivo CSV
def load_odometry_from_csv(file_name):
    t_values = []
    x_values = []
    with open(file_name, mode='r') as file:
        reader = csv.reader(file)
        next(reader) #Pula o cabeçalho (timestamp, x, y)
        for row in reader:
            t = float(row[0])
            x = float(row[1])
            
            t_values.append(t)
            x_values.append(x)

    return t_values, x_values

t_fast_lio2, x_fast_lio2 = load_odometry_from_csv('/home/gabriel/odometria_fast_lio2_1m_gt.csv')
t_husky, x_husky = load_odometry_from_csv('/home/gabriel/odometria_husky_1m_gt.csv')
t_ekf, x_ekf = load_odometry_from_csv('/home/gabriel/odometria_ekf_1m_gt.csv')
t_gt, x_gt = load_odometry_from_csv('/home/gabriel/odometria_gt_1m.csv')

timestamp_inicial_fast_lio2 = t_fast_lio2[0]
timestamp_inicial_husky = t_husky[0]
timestamp_inicial_ekf = t_ekf[0]
timestamp_inicial_gt = t_gt[0]

t_normalizado_fast_lio2 = [t - timestamp_inicial_fast_lio2 for t in t_fast_lio2]
t_normalizado_husky = [t - timestamp_inicial_husky for t in t_husky]
t_normalizado_ekf = [t - timestamp_inicial_ekf for t in t_ekf]
t_normalizado_gt = [t - timestamp_inicial_gt for t in t_gt]

plt.plot(t_normalizado_fast_lio2, x_fast_lio2, 'r-', label='Coordenada x estimada pelo Fast-LIO 2')
plt.plot(t_normalizado_husky, x_husky, 'b-', label='Coordenada x estimada pela odometria de rodas')
plt.plot(t_normalizado_ekf, x_ekf, 'g-', label='Coordenada x estimada pelo filtro de Kalman estendido')
plt.plot(t_normalizado_gt, x_gt, 'k-', label='Coordenada x do ground truth')

# Configuração da legenda dos eixos e do título do gráfico
plt.xlabel('t')
plt.ylabel('x')
plt.title('Comparação das estimações da coordenada x ao longo do tempo')

plt.legend() # Exibir a legenda
plt.show() # Mostra o gráfico
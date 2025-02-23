import csv
import matplotlib.pyplot as plt 

# Função para carregar os dados do arquivo CSV
def load_odometry_from_csv(file_name):
    y_values = []
    x_values = []
    with open(file_name, mode='r') as file:
        reader = csv.reader(file)
        next(reader) #Pula o cabeçalho (timestamp, x, y)
        for row in reader:
            x = float(row[1])
            y = float(row[2])

            x_values.append(x)
            y_values.append(y)

    return y_values, x_values

# Função para carregar os dados do arquivo CSV
y_fast_lio2, x_fast_lio2 = load_odometry_from_csv('/home/gabriel/odometria_fast_lio2_1m_gt.csv')
y_husky, x_husky = load_odometry_from_csv('/home/gabriel/odometria_husky_1m_gt.csv')
y_ekf, x_ekf = load_odometry_from_csv('/home/gabriel/odometria_ekf_1m_gt.csv')
y_gt, x_gt = load_odometry_from_csv('/home/gabriel/odometria_gt_1m.csv')

plt.figure() # Cria o gráfico

plt.plot(x_fast_lio2, y_fast_lio2, 'r-', label='Pose 2D do robô estimada pelo Fast-LIO 2')
plt.plot(x_husky, y_husky, 'b-', label='Pose 2D do robô estimada pela odometria de rodas')
plt.plot(x_ekf, y_ekf, 'g-', label='Pose 2D do robô estimada pelo filtro de Kalman estendido')
plt.plot(x_gt, y_gt, 'k-', label='Pose 2D do robô pelo ground truth')

plt.xlabel('x')
plt.ylabel('y')
plt.title('Comparação das estimações da pose 2D do robô ao longo do tempo')

plt.legend() # Exibir a legenda
plt.show() # Mostra o gráfico
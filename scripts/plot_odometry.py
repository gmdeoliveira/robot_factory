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

t_fast_lio2, x_fast_lio2 = load_odometry_from_csv('/home/gabriel/odometria_fast_lio2_5m.csv')
t_husky, x_husky = load_odometry_from_csv('/home/gabriel/odometria_husky_5m.csv')

timestamp_inicial_fast_lio2 = t_fast_lio2[0]
timestamp_inicial_husky = t_husky[0]

t_normalizado_fast_lio2 = [t - timestamp_inicial_fast_lio2 for t in t_fast_lio2]
t_normalizado_husky = [t - timestamp_inicial_husky for t in t_husky]

plt.figure() # Cria o gráfico

plt.plot(t_normalizado_fast_lio2, x_fast_lio2, 'r-', label='Coordenada x estimada pelo Fast-LIO 2')

plt.plot(t_normalizado_husky, x_husky, 'b-', label='Coordenada x estimada pela odometria de rodas')


# Configuração da legenda dos eixos e do título do grbfico
plt.xlabel('t')
plt.ylabel('x')
plt.title('Comparação das estimações da coordenada x ao longo do tempo')

plt.legend() # Exibir a legenda
plt.show() # Mostra o gráfico
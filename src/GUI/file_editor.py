import os
import yaml
import matplotlib.pyplot as plt

current_folder = os.path.dirname(os.path.abspath(__file__))
yaml_file = os.path.join(current_folder, "statistics.yaml")
# Función para leer desde un archivo YAML
def load_yaml():
    with open(yaml_file, 'r') as f:
        datos = yaml.safe_load(f)
    return datos

# Función para escribir en un archivo YAML
def write_yaml(datos):
    with open(yaml_file, 'w') as f:
        yaml.safe_dump(datos, f)


def add_object_run(name):
    data = load_yaml()
    # print(data)
    if name in data.keys():
        data[name]['runs'] +=1
    else:
        data[name] = {
            'recognized': 0,
            'grasped' : 0,
            'runs' : 1
        }
    write_yaml(data)

def add_object_recog(name):
    data = load_yaml()
    if name in data.keys():
        data[name]['recognized'] +=1
    write_yaml(data)

def add_object_grasp(name):
    data = load_yaml()
    if name in data.keys():
        data[name]['grasped'] +=1
    write_yaml(data)

def view_stats():
    datos = load_yaml()

    # Lista de colores para las barras
    colores = ['blue', 'red', 'green']
    etiquetas = ['Runs', 'Recognized', 'Grasped']

    # Configura el gráfico
    fig, ax = plt.subplots(figsize=(10, 6))
    ancho_barra = 0.25

    # Procesa los datos y grafica las barras
    for i, (objeto, valores) in enumerate(datos.items()):
        ax.bar(i - ancho_barra - 0.02, valores['runs'], color=colores[0], width=ancho_barra, label='Runs')
        ax.bar(i, valores['recognized'], color=colores[1], width=ancho_barra, label='Recognized')
        ax.bar(i + ancho_barra + 0.02, valores['grasped'], color=colores[2], width=ancho_barra, label='Grasped')

        # Añade la efectividad encima de las barras recognized y grasped
        if valores['runs'] != 0:  
            ax.text(i, valores['recognized'] + 0.1, f"{valores['recognized']/valores['runs']*100:.2f}%", ha='center', va='bottom', color='black', rotation=45)
            ax.text(i + ancho_barra, valores['grasped'] + 0.1, f"{valores['grasped']/valores['runs']*100:.2f}%", ha='center', va='bottom', color='black', rotation=45)

    # Añade las etiquetas y los nombres de los objetos
    ax.set_xlabel('Objetos')
    ax.set_ylabel('Cantidad')
    ax.set_title('Estadisticas pickup por objeto')
    ax.set_xticks(range(len(datos)))
    ax.set_xticklabels(datos.keys())

    # Añade la leyenda
    ax.legend(etiquetas, loc='upper right')

    plt.tight_layout()
    plt.show()


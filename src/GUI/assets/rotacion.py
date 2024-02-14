from PIL import Image

# Ruta de la imagen original y prefijo para los nombres de las imágenes rotadas
original_image_path = '/home/rusanrod/giro.png'
prefix = 'arrow'
sufix = ["r", "l"]
# sufix = ['left', 'down', 'right', 'up']
# Tamaño deseado para las imágenes redimensionadas
new_size = (25, 25)

# Rotar la imagen original tres veces en incrementos de 90 grados y guardar las imágenes rotadas

for i in range(1):
    angle = (i + 1) * 90  # Calcular el ángulo de rotación
    image_name = f'{prefix}_{sufix[i]}.png'  # Nombre del archivo para la imagen rotada
    with Image.open(original_image_path) as img:
        resized_img = img.resize(new_size)
        # mirroed_img = resized_img.transpose(Image.FLIP_LEFT_RIGHT)
        # rotated_img = resized_img.rotate(angle, expand=True)
        resized_img.save(image_name)
        print(f'Imagen rotada {angle} grados guardada como {image_name}')

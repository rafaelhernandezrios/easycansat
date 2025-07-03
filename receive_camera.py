import serial
import time

ser = serial.Serial('COM11', 115200, timeout=5)  # Cambia el puerto si es necesario
img_count = 0

while True:
    # Espera el byte de inicio
    while ser.read(1) != b'\xAA':
        pass
    # Lee el tamaño de la imagen (2 bytes)
    size_bytes = ser.read(2)
    img_size = (size_bytes[0] << 8) | size_bytes[1]
    # Lee la imagen
    img_data = b''
    while len(img_data) < img_size:
        img_data += ser.read(img_size - len(img_data))
    # Guarda la imagen
    filename = f'captura_{img_count}.jpg'
    with open(filename, 'wb') as f:
        f.write(img_data)
    print(f'Imagen guardada: {filename}')
    img_count += 1
    # time.sleep(0.1)  # Descomenta si quieres limitar la tasa de guardado 
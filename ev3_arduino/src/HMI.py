import rospy
from std_msgs.msg import UInt8
import tkinter as tk
from PIL import Image, ImageTk
from pynput import keyboard

__author__ = "Felipe Cruz"
__credits__ = ["Felipe Cruz"]
__email__ = "fcruzv@unal.edu.co"
__status__ = "Test"

# Control EV3
comandoEnviado = 0
comandoEV3 = UInt8()
comandoEV3.data = 0

# Crear la ventana de la interfaz gráfica
root = tk.Tk()
root.title("Control Lego EV3")

# Crear una función para PARAR
def STOP():
    exit()

# Colocar nombres y titulo
tk.Label(root, text="HMI para controlar robot EV3").pack()
tk.Label(root, text="Felipe Cruz").pack()

# Crear espacios para mostrar info
color_label = tk.Label(root, text="Color: ")
color_label.pack()
contacto_label = tk.Label(root, text="Contacto: ")
contacto_label.pack()

# Crear un botón para parar
move_button = tk.Button(root, text="PARAR", command=STOP)
move_button.pack()

# Definir la función callback para actualizar los valores en tiempo real
def subscriber(data):
    if(data.data == 20):
        color_label.config(text=f"Color: Sin Color")
    elif(data.data == 21):
        color_label.config(text=f"Color: Negro")
    elif(data.data == 22):
        color_label.config(text=f"Color: Azul")
    elif(data.data == 23):
        color_label.config(text=f"Color: Verde")
    elif(data.data == 24):
        color_label.config(text=f"Color: Amarillo")
    elif(data.data == 25):
        color_label.config(text=f"Color: Rojo")
    elif(data.data == 26):
        color_label.config(text=f"Color: Blanco")
    elif(data.data == 27):
        color_label.config(text=f"Color: Café")
    elif(data.data == 10):
        contacto_label.config(text=f"Contacto: No")
    elif(data.data == 11):
        contacto_label.config(text=f"Contacto: Sí")
    else:
        pass

def keyboardPress(key):
    global comandoEV3
    if key == keyboard.Key.up:
        comandoEV3.data = 51
    elif key == keyboard.Key.right:
        comandoEV3.data = 52
    elif key == keyboard.Key.left:
        comandoEV3.data = 53
    elif key == keyboard.Key.down:
        comandoEV3.data = 54
    else:
        comandoEV3.data = 50

def process():
    global comandoEV3
    global comandoEnviado
    rospy.Subscriber("sensores", UInt8, subscriber)
    publisher = rospy.Publisher("motores", UInt8, queue_size=10)
    if(comandoEnviado != comandoEV3.data):
        comandoEnviado = comandoEV3.data
        publisher.publish(comandoEV3)
   
def process_ui():
    # Revisar nodo activo
    if rospy.is_shutdown():
        return
    
    process() # Leer datos
    root.after(1000, process_ui)  # Actualizar

# Funcion principal
if __name__ == '__main__':
    # Inicializar nodo
    rospy.init_node('nodo_UI', anonymous=True)

    # Inicializar hilo del teclado
    keyboardManager = keyboard.Listener(on_press=keyboardPress)
    keyboardManager.start()

    # Iniciar un hilo para procesamiento de la UI
    process_ui()

    root.mainloop() # HMI
import tkinter as tk
from tkinter import ttk
import serial
from serial.tools import list_ports

window = tk.Tk() # crea la ventana apartir de un metodo de tk
window.title('Control de Inclinación con Mouse') #titulo de la ventana

screen_w = 1000#int(window.winfo_screenwidth()*0.98) # Informacion de la pantalla
screen_h = 750#int(window.winfo_screenheight()*0.9)
window.geometry(f'{screen_w}x{screen_h}+200+0') #ajustado para una pantalla FULL HD
window.resizable(False, False)

class Option_btns(ttk.Frame):
  def __init__(self, parent, Btn_UP_name, Btn_DOWN_name, height, font_text):
    super().__init__(parent)
    super().configure(relief= 'groove', border= 5)#añade un borde para ver los limites
    self.parent = parent
    self.height = height
    #-- configurando grid --
    self.config(height=self.height)
    self.rowconfigure((0,1), weight=1)
    self.columnconfigure(0, weight=1)

    #--Elementos (widgets) --
    self.Btn_UP = tk.Button(self, text= Btn_UP_name, font=font_text)
    self.Btn_DOWN = tk.Button(self, text= Btn_DOWN_name, font=font_text)

    #-- empaquedato en grid --
    self.Btn_UP.grid(row= 0, column=0, sticky='swe', ipady= int(self.height*0.1) , pady=10)
    self.Btn_DOWN.grid(row= 1, column=0, sticky='nwe',ipady= int(self.height*0.1), pady=10)

  def config_fuctions(self,Btn_UP_func, Btn_DOWN_func):
    self.Btn_UP.config(command= Btn_UP_func)
    self.Btn_DOWN.config(command= Btn_DOWN_func)

  def config_btns_names(self,Btn_UP_name, Btn_DOWN_name):
    self.Btn_UP.config(text= Btn_UP_name)
    self.Btn_DOWN.config(text= Btn_DOWN_name)

  def disabled_btns(self):
    self.Btn_UP.config(state='disabled')
    self.Btn_DOWN.config(state='disabled')

  def enabled_btns(self):
    self.Btn_UP.config(state='normal')
    self.Btn_DOWN.config(state='normal')

class config_serial_tab(ttk.Frame):
  def __init__(self, parent, size):
    super().__init__(parent)
    super().configure(relief= 'groove', border= 5)#añade un borde para ver los limites
    self.main_size = size
    self.parent = parent
    self.conected = False

    self.available_serial_ports = [] #lista de puertos disponibles
    self.conexion_test_callback = ()
    
    self.cmd_port = None
    self.id_task_readSerial = None
    #-- configurando grid --
    self.columnconfigure((0,1), weight=1)
    self.columnconfigure(2, weight=2)
    self.rowconfigure((0,2), weight=1)
    self.rowconfigure(1, weight= 2)

    #-- Elementos (widgets) --
    self.serial_ports_combobox = ttk.Combobox(self, state= 'readonly')
    self.serial_config_btns = Option_btns(self, Btn_UP_name= 'SELECCIÓN\nDE\nPUERTO\nSERIAL',
                                          Btn_DOWN_name= 'CONECTAR',
                                          height= self.main_size[1],
                                          font_text='Arial 8 bold')
    self.serial_command_log_text = tk.Text( self, width= int(self.main_size[0]*0.1), 
                                            height=int(self.main_size[1]*0.5))
    
    #-- empaquedato en grid --
    self.serial_config_btns.grid(row=1,column=0, sticky='nswe')
    self.serial_ports_combobox.grid(row=1, column=1, sticky='swe', pady=int(size[1]*0.4))
    self.serial_command_log_text.grid(row=1, column=2)

    #-- Eventos --
    self.serial_config_btns.config_fuctions(Btn_UP_func= self.Update_serial_ports,
                                            Btn_DOWN_func= self.doSerialConection)
    self.Update_serial_ports()
    if self.serial_ports_combobox.current() != None:
      self.after(100,self.doSerialConection)

  def Update_serial_ports(self):
    self.available_serial_ports = [port.device for port in list_ports.comports()]
    self.serial_ports_combobox["values"] = self.available_serial_ports

  def doSerialConection(self):
    
    puerto_seleccionado = self.serial_ports_combobox.get()
    if puerto_seleccionado:
      try:
        self.cmd_port = serial.Serial(puerto_seleccionado, 115200)
        self.serial_command_log_text.config(state="normal")
        self.serial_command_log_text.delete("1.0", tk.END)
        self.serial_config_btns.Btn_UP.config(state='disabled')#self.boton_conectar.config(state="disabled")
        self.serial_config_btns.Btn_DOWN.configure(text='DESCONECTAR', command= self.disconectSerial)
        self.serial_command_log_text.insert(tk.END, "Conexión establecida con el puerto serial: " + puerto_seleccionado + "\n")
        #self.leer_puerto_serial()
        self.conected = True
        self.cmd_port.write('z90\n'.encode())
        self.read_serial_port()

      except serial.SerialException:
        self.serial_command_log_text.insert(tk.END, "Error al conectar al puerto " + puerto_seleccionado + "\n")

  def disconectSerial(self):
    puerto_seleccionado = self.serial_ports_combobox.get()
    #if self.cmd_port is None and self.cmd_port.is_open:
    #  try:
    if self.cmd_port is not None and self.cmd_port.is_open:
      self.cmd_port.close()
      self.serial_config_btns.Btn_UP.config(state='normal')
      self.serial_config_btns.Btn_DOWN.configure(text='CONECTAR', command= self.doSerialConection)
      if self.id_task_readSerial is not None:
        self.after_cancel(self.id_task_readSerial)
      self.serial_command_log_text.insert(tk.END, "Desconectado de puerto serial " + puerto_seleccionado + "\n")
      self.conected = False
      #except:
      #  self.serial_command_log_text.insert(tk.END, "No se pudo, desconectar del " + puerto_seleccionado + "\n")

  def read_serial_port(self):
    if self.cmd_port is None or not self.cmd_port.is_open:
      return
    try:
      if self.cmd_port.in_waiting > 0:
        dato = self.cmd_port.readline().decode().strip()
        if not dato.startswith('x') and not dato.startswith('y'):
          self.serial_command_log_text.insert(tk.END, dato + "\n")
    except serial.SerialException:
      self.serial_command_log_text.insert(tk.END, "Error al leer el puerto serial\n")
      
    self.id_task_readSerial = self.after(20, self.read_serial_port)

def map(valor, min_rango_original,max_rango_original , min_nuevo_rango, max_nuevo_rango):
  valor = float(valor)
  min_rango_original = float(min_rango_original)
  max_rango_original = float(max_rango_original)
  min_nuevo_rango = float(min_nuevo_rango)
  max_nuevo_rango = float(max_nuevo_rango)
  res = (valor - min_rango_original) * (max_nuevo_rango - min_nuevo_rango) / (max_rango_original - min_rango_original) + min_nuevo_rango
  return '{:.2f}'.format(res)

tabs = ttk.Notebook(window, width= screen_w, height= screen_h)

pad = tk.Frame(tabs, width=screen_w, height=screen_h)

serial_tab = config_serial_tab(tabs, size=(screen_w,screen_h))


tabs.add(serial_tab, text='Configuracion Serial')
tabs.add(pad,text='PAD para control Inclinacón')

tabs.pack(expand=True)
def evento_follow_pos(event):
  x_incli = map(event.x,0,screen_w,-15,15)
  y_incli = map(event.y,0,screen_h,15,-15)
  if serial_tab.conected:
    serial_tab.cmd_port.write(f'x{y_incli}\n'.encode())
    serial_tab.cmd_port.write(f'y{x_incli}\n'.encode())
  else:
    print(f'x: {x_incli}, y: {y_incli}')

def evento_reset_pos(event):
  serial_tab.cmd_port.write(f'x{0}\n'.encode())
  serial_tab.cmd_port.write(f'y{0}\n'.encode())

pad.bind('<Motion>',evento_follow_pos)
pad.bind('<Leave>', evento_reset_pos)
window.mainloop()
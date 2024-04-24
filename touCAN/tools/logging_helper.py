from tkinter import *
from datetime import datetime
import csv
  
window = Tk() 
window.title('Signals to Log') 
window.geometry("300x400")
  
# for scrolling vertically 
yscrollbar = Scrollbar(window) 
yscrollbar.pack(side = RIGHT, fill = Y) 
  
label = Label(window, 
              text = "", 
              font = ("Times New Roman", 10),  
              padx = 120, pady = 10) 
label.pack() 
listbx = Listbox(window, selectmode = "multiple",  
               yscrollcommand = yscrollbar.set) 
  
# Widget expands horizontally and  
# vertically by assigning both to 
# fill option 
listbx.pack(padx = 10, pady = 10, 
          expand = YES, fill = "both") 
  
# Attach listbox to vertical scrollbar 
yscrollbar.config(command = listbx.yview) 

def RefreshCallBack():
   listbx.delete(0,'end')
   with open("../dataBroker.h", 'r') as brokerfile:
    for line in csv.reader(brokerfile, delimiter=';'):
        if len(line)<=0:
            continue
        index = line[0].find("extern BrokerData ")
        if (not index < 0):
            string = line[0]
            listbx.insert(END, string[index+18:])
   
   
B = Button(window, text ="Refresh", command = RefreshCallBack)
B.place(x=10,y=15)
selectedItems = []
def GenerateCallBack():
   # Get the indices of all the selected items
   selectedIndices = listbx.curselection()
   selectedItems = []
   # Loop over the selected indices and get the values of the selected items
   for index in selectedIndices:
    selectedItems.append(listbx.get(index))
   
   with open("LoggingConfig.h", 'w+') as config:
    config.write("\n//Generated {} with logging_helper.py for EV Kartz Kettering University\n//Henry Grasman\n\n".format(datetime.now()))
    config.write("#ifndef LOGGING_CONFIG\n#define LOGGING_CONFIG\n\n#include \"FS.h\"\n\n")
    
    #flush logic
    config.write("""#define FLUSH_RATE 50
uint8_t flushCounter = 0;

inline void logging_flush_buffer(File logfile){
  if (flushCounter++ > FLUSH_RATE){
    WRAP_SPI_MUTEX(logfile.flush();, portMAX_DELAY)
    flushCounter = 0;
  }
}

""")
    
    #populate header function
    config.write("inline void logging_write_header(File logfile){\n")
    config.write("WRAP_SPI_MUTEX(logfile.print(\"LeSDLR_t_currentTime\");, portMAX_DELAY)\n")
    for item in selectedItems:
        config.write("WRAP_SPI_MUTEX(logfile.print(\", {}\");, portMAX_DELAY)\n".format(item))
    config.write("WRAP_SPI_MUTEX(logfile.print(\"\\n\");, portMAX_DELAY)\nWRAP_SPI_MUTEX(logfile.flush();,portMAX_DELAY)\n}\n\n")
    
    #populate logger write function
    config.write("inline void logging_write_line(File logfile){\n")
    local = []
    config.write("double LeSDLR_t_currentTime = (double)esp_timer_get_time() / 100000.0;\n")
    for item in selectedItems:
        new_local = "LeSDLR" + item[6:]
        config.write("double {} = {}.getValue();\n".format(new_local, item))
        local.append(new_local)
    config.write("\nWRAP_SPI_MUTEX(\\\n")
    config.write("logfile.print(LeSDLR_t_currentTime);\\\n")
    for item in local:
        config.write("logfile.print(\", \"); logfile.print({});\\\n".format(item))
    config.write("logfile.print(\"\\n\");,portMAX_DELAY)\n}\n\n")
    
    config.write("#endif")

B = Button(window, text ="Generate Logger Code", command = GenerateCallBack)
B.place(x=70,y=15)

RefreshCallBack()

window.mainloop() 
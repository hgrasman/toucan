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
listbx = Listbox(window, selectmode=EXTENDED,  
               yscrollcommand = yscrollbar.set) 
  
# Widget expands horizontally and  
# vertically by assigning both to 
# fill option 
listbx.pack(padx = 10, pady = 10, 
          expand = YES, fill = "both") 
  
# Attach listbox to vertical scrollbar 
yscrollbar.config(command = listbx.yview) 

precisions = []
def RefreshCallBack():
   listbx.delete(0,'end')
   with open("../dataBroker.h", 'r') as brokerfile:
    for line in csv.reader(brokerfile):
        if len(line)<=0:
            continue
        index = line[0].find("extern BrokerData ")
        end_index = line[0].find(";")
        if (index >= 0 and end_index >= 0):
            string = line[0]
            listbx.insert(END, string[index+18:end_index])
            
            index = line[0].find("LOGPRECISION")
            end_index = line[0].find(".", index)
            if (index >= 0 and end_index >= 0):
                precisions.append(int(string[index+13:end_index]))
            else:
                precisions.append(0) #default

   
   
B = Button(window, text ="Refresh", command = RefreshCallBack)
B.place(x=10,y=15)
selectedItems = []


def GenerateCallBack():
   # Get the indices of all the selected items
   selectedIndices = listbx.curselection()
   selectedItems = []
   indexes = []
   # Loop over the selected indices and get the values of the selected items
   for index in selectedIndices:
    selectedItems.append(listbx.get(index))
    indexes.append(index)
   
   with open("LoggingConfig.h", 'w+') as config:
    config.write("\n//Generated {} with logging_helper_2.py for EV Kartz Kettering University\n//Henry Grasman\n\n".format(datetime.now()))
    config.write("#ifndef LOGGING_CONFIG\n#define LOGGING_CONFIG\n\n#include \"FS.h\"\n\n")
    
    #local names
    local = []
    for item in selectedItems:
        new_local = "LeSDLR" + item[6:]
        local.append(new_local)
    
    #flush logic
    config.write("""#define LOG_RATE 100
#define FLUSH_RATE 10
uint8_t flushCounter = 0;

""")
    #struct logic
    config.write("struct loggingData{\n  double LeSDLR_t_currentTime;\n")
    for item in local:
        config.write("  double {};\n".format(item))
    config.write("  double LeSDLR_t_endTime;\n")
    config.write("}loggingMessage, dataToLog;\n\n")
    
    #queue
    config.write("""QueueHandle_t loggingQueue = xQueueCreate( 16, sizeof( struct loggingData ) );

""")

    #flush logic
    config.write("""inline void logging_flush_buffer(File logfile){
  if (flushCounter++ > FLUSH_RATE){
    logfile.flush();
    flushCounter = 0;
  }
}

""")

    #queuedata logic
    config.write("inline bool logging_queue_data(void){\n")
    config.write("  loggingMessage.LeSDLR_t_currentTime = (double)esp_timer_get_time() / 1000000.0;\n")
    for item in selectedItems:
        new_local = "LeSDLR" + item[6:]
        config.write("  loggingMessage.{} = {}.getValue();\n".format(new_local, item))
    config.write("  loggingMessage.LeSDLR_t_endTime = (double)esp_timer_get_time() / 1000000.0;\n")
    config.write("\n  return (xQueueSend( loggingQueue, ( void * ) &loggingMessage, portMAX_DELAY ) == pdTRUE);\n}\n\n")
    
    #populate header function
    config.write("inline void logging_write_header(File logfile){\n")
    config.write("  logfile.print(\"LeSDLR_t_currentTime\");\n")
    for item in selectedItems:
        config.write("  logfile.print(\", {}\");\n".format(item))
    config.write("  logfile.print(\", LeSDLR_t_endTime\");\n")
    config.write("  logfile.print(\"\\n\");\n  logfile.flush();\n}\n\n")
    
    #populate logger write function
    config.write("inline void logging_write_line(File logfile, struct loggingData *pdataToLog){\n")
    config.write("  logfile.print(pdataToLog->LeSDLR_t_currentTime, 4);\n")
    for i, item in enumerate(local):
        config.write("  logfile.print(\", \"); logfile.print(pdataToLog->{}, {});\n".format(item, precisions[indexes[i]]))
    config.write("  logfile.print(\", \"); logfile.print(pdataToLog->LeSDLR_t_endTime, 4);\n")
    config.write("  logfile.print(\"\\n\");\n}\n\n")
    
    config.write("#endif")
    
    window.destroy()
B = Button(window, text ="Generate Logger Code", command = GenerateCallBack)
B.place(x=70,y=15)


RefreshCallBack()

window.mainloop() 